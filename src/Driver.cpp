#include "Driver.hpp"

using namespace qualisys;

//
// The implementation of this protocol is based on the information in the
// QTM Real-time Server Protocol Documentation Version 1.9
//
// Document Number: QDEV-QTM-EXTERN-SDK_RT-QTM_RT_SERVER_V_1_9
// Revision: 1.0
// Date: 2011-06-08
//
//

const size_t qualisys::Driver::MAX_MARKERS = 64;

struct QTMHeader
{
    enum EPacketType
    {
        PacketError       = 0,
        PacketCommand     = 1,
        PacketXML         = 2,
        PacketData        = 3,
        PacketNoMoreData  = 4,
        PacketC3DFile     = 5,
        PacketEvent       = 6,
        PacketDiscover    = 7,
        PacketNone        = 8
    };

    uint32_t size;
    uint32_t type;

    /** 
     * check the QTM packet header
     * will throw if the packet header is not valid
     */
    void checkValid() const
    {
	if( size < 8 )
	    throw std::runtime_error("QTM Protocol: invalid packet size.");

	if( type > 8 )
	    throw std::runtime_error("QTM Protocol: Invalid packet type.");
    }

    size_t getSize() const
    {
	return size;
    }

    size_t getPayloadSize() const
    {
	return getSize() - sizeof( QTMHeader );
    }

    EPacketType getType() const
    {
	return static_cast<EPacketType>( type );
    }

    QTMHeader() {}

    QTMHeader( EPacketType type, size_t size )
	: size( size ), type( type )
    {
    }
} __attribute__ ((__packed__));

struct ComponentHeader
{
    enum EComponentType 
    {
        Component3d            = 1,
        Component3dNoLabels    = 2,
        ComponentAnalog        = 3,
        ComponentForce         = 4,
        Component6d            = 5,
        Component6dEuler       = 6,
        Component2d            = 7,
        Component2dLin         = 8,
        Component3dRes         = 9,
        Component3dNoLabelsRes = 10,
        Component6dRes         = 11,
        Component6dEulerRes    = 12,
        ComponentAnalogSingle  = 13,
        ComponentImage         = 14,
        ComponentForceSingle   = 15,
        ComponentNone          = 16
    };

    EComponentType getType() const
    {
	if( type > 16 )
	    throw std::runtime_error("QTMComponent: Invalid component type");

	return static_cast<EComponentType>(type);
    }

    uint32_t size;
    uint32_t type;
    uint32_t marker_count;
    uint16_t drop_rate;
    uint16_t out_of_sync_rate;
} __attribute__ ((__packed__));

struct Marker6d
{
    float x,y,z;
    float rot[9];
} __attribute__ ((__packed__));

Driver::Driver()
    : iodrivers_base::Driver( sizeof( QTMHeader ) + sizeof( ComponentHeader ) + MAX_MARKERS * sizeof( Marker6d ) )
{}

bool isLittleEndian()
{
    int x = 1;
    return (*(char *)&x == 1);
}

void Driver::connect( const std::string& host, const unsigned int base_port )
{
    // add offset to base port based on endianess
    unsigned int port = isLittleEndian() ? base_port + 1 : base_port + 2;

    // construct uri string
    openTCP( host, port );

    // get the welcome string
    std::string result;
    if( !readQTMString( result ) )
    {
	throw std::runtime_error("Connect failed. Missing welcome message.");
    }

    setQTMVersion( 1, 9 );
}

void Driver::startStreamData()
{
    // start streaming of data frames 
    std::stringstream ss;
    ss << "StreamFrames";
    ss << " 6D";
    writeQTMCommand( ss.str() );
}

void Driver::stopStreamData()
{
    // stop streaming of data frames 
    std::stringstream ss;
    ss << "StreamFrames";
    ss << " Stop";
    writeQTMCommand( ss.str() );
}

bool Driver::getTransform( base::Time& time, base::Affine3d& transform, int marker_idx )
{
    const size_t buf_size = sizeof( QTMHeader ) + sizeof( ComponentHeader ) + MAX_MARKERS * sizeof( Marker6d );
    uint8_t buf[buf_size];

    int res = readPacket( buf, buf_size );
    if( res <= 0 )
	return false;

    // check packet header
    QTMHeader *header = reinterpret_cast<QTMHeader*>( buf );
    if( header->getType() != QTMHeader::PacketData )
	return false;

    // check component header
    ComponentHeader *cheader = reinterpret_cast<ComponentHeader*>( buf + sizeof( QTMHeader) );
    if( cheader->getType() != ComponentHeader::Component6d )
	return false;

    // for now, we only return the marker with the given index
    if( marker_idx >= static_cast<int>(cheader->marker_count) )
	throw std::runtime_error( "getTransform: invalid marker index." );
    
    // get requested body id from the struct
    Marker6d *c6d = reinterpret_cast<Marker6d*>( 
	    buf + sizeof( QTMHeader) 
	    + sizeof( ComponentHeader) 
	    + marker_idx * sizeof( Marker6d) );
    
    // set the result values
    time = base::Time::now();
    transform.makeAffine();
    transform.linear() = Eigen::Map<Eigen::Matrix3f>( c6d->rot ).cast<double>();
    transform.translation() = Eigen::Map<Eigen::Vector3f>( &c6d->x ).cast<double>();

    return true;
}

/** 
 * reads a packet from the stream and interprets it as a string 
 */
bool Driver::readQTMString( std::string& qtmstring )
{
    const size_t buf_size = 10000;
    uint8_t buf[buf_size];

    int res = readPacket( buf, buf_size );
    if( res <= 0 )
	return false;

    if( res >= static_cast<int>(buf_size) )
	throw std::runtime_error( "readQTMString: Buffer not large enough." );

    QTMHeader *header = reinterpret_cast<QTMHeader*>(buf);

    if( header->getType() != QTMHeader::PacketCommand )
	throw std::runtime_error( "readQTMString: Wrong packet type. " + header->getType() );

    qtmstring = std::string( reinterpret_cast<char*>(buf + sizeof(QTMHeader)), header->getPayloadSize() - 1 );

    return true;
}

/**
 * write a version command 
 */
bool Driver::setQTMVersion( int major, int minor )
{
    // send a version command
    std::stringstream ss;
    ss << "Version " << major << "." << minor;
    writeQTMCommand( ss.str() );

    // assume that
    std::stringstream ssr;
    ssr << "Version set to " << major << "." << minor;
    std::string result;
    if( readQTMString( result ) )
    {
	if( result == ssr.str() ) 
	    return true;
	else
	    throw std::runtime_error("setQTMVersion: failed. Response: " + result );
    }

    throw std::runtime_error("setQTMVersion: failed to read version." );
}

void Driver::writeQTMCommand( const std::string& cmd )
{
    writeQTMCommand( reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.size() );
}

void Driver::writeQTMCommand( uint8_t const* buffer, size_t size )
{
    // write the header first
    QTMHeader header( QTMHeader::PacketCommand, size + sizeof(QTMHeader) );

    // write out the header first
    writePacket( reinterpret_cast<uint8_t*>(&header), sizeof(QTMHeader) );

    // and then the payload
    writePacket( buffer, size );
}

int Driver::extractPacket( uint8_t const* buffer, size_t size) const
{
    // The header for each packet are 8-bytes,
    // 4 bytes - size of the packet including header
    // 4 bytes - type of packet
    
    // we don't do anything until we have received a full header
    if( size < sizeof(QTMHeader) )
	return 0;

    const QTMHeader *header = reinterpret_cast<const QTMHeader*>(buffer);
    
    // the best way to correctly identify a valid packet is by 
    // the type. As of protocol version 1.9, there are 8 different 
    // types of packages (see enum).
    //
    // Method will throw if the header is not valid.
    header->checkValid();

    // here we assume a valid packet
    // but it might not be complete yet
    if( size >= header->getSize() )
	return header->getSize();
    else
	return 0;
}
