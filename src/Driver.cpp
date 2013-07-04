#include "Driver.hpp"
#include <tinyxml.h>

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

Driver::Driver()
    : iodrivers_base::Driver( sizeof( QTMHeader ) + sizeof( ComponentHeader ) + MAX_MARKERS * sizeof( Marker6d ) ),
    body_idx(-1)
{
    buffer_size = sizeof( QTMHeader ) + sizeof( ComponentHeader ) + MAX_MARKERS * sizeof( Marker6d ); 
    buffer = new uint8_t[buffer_size];
}

Driver::~Driver()
{
    delete[] buffer;
}

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

void Driver::startStreamData( const std::string& label )
{
    body_idx = -1;

    std::vector<std::string> labels;
    loadParameters(labels);
    for( size_t i=0; i<labels.size(); i++ )
    {
	if( labels[i] == label )
	    body_idx = i;
    }

    if( body_idx < 0 )
	throw std::runtime_error("Could not find specified body label.");

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

void Driver::loadParameters( std::vector<std::string>& labels )
{
    // send a version command
    std::stringstream ss;
    ss << "GetParameters 6D";
    writeQTMCommand( ss.str() );

    // load the resulting xml parameters
    if( !readQTMPacket( QTMHeader::PacketXML ) )
	throw std::runtime_error("could not read parameter response.");

    // parse the xml
    TiXmlDocument xmldoc;
    xmldoc.Parse( reinterpret_cast<char*>(buffer + sizeof(QTMHeader)) );

    // iterate over the bodies in the xml struct
    TiXmlNode *the6d = xmldoc.FirstChild()->FirstChild("The_6D");
    TiXmlNode *body = 0;
    while( (body = the6d->IterateChildren( "Body", body )) )
    {
	std::string name = body->FirstChild( "Name" )->Value();
	labels.push_back( name );
    }
}

bool Driver::getTransform( base::Time& time, base::Affine3d& transform )
{
    if( body_idx < 0 )
	throw std::runtime_error("No valid marker/body index has been selected using the startSream call.");

    if( !readQTMPacket( QTMHeader::PacketData ) )
	return false;

    // check component header
    ComponentHeader *cheader = reinterpret_cast<ComponentHeader*>( buffer + sizeof( QTMHeader) );
    if( cheader->getType() != ComponentHeader::Component6d )
	return false;

    // for now, we only return the marker with the given index
    if( body_idx >= static_cast<int>(cheader->marker_count) )
	throw std::runtime_error( "getTransform: invalid marker index." );
    
    // get requested body id from the struct
    Marker6d *c6d = reinterpret_cast<Marker6d*>( 
	    buffer + sizeof( QTMHeader) 
	    + sizeof( ComponentHeader) 
	    + body_idx * sizeof( Marker6d) );
    
    // set the result values
    time = base::Time::now();
    transform.makeAffine();
    transform.linear() = Eigen::Map<Eigen::Matrix3f>( c6d->rot ).cast<double>();
    transform.translation() = Eigen::Map<Eigen::Vector3f>( &c6d->x ).cast<double>();

    return true;
}

bool Driver::readQTMPacket( QTMHeader::EPacketType type )
{
    // there might be event packages in the meantime, that 
    // we ignore for now, so skip those
    QTMHeader *header = reinterpret_cast<QTMHeader*>(buffer);
    do
    {
	int res = readPacket( buffer, buffer_size );
	if( res <= 0 )
	    return false;

	if( res >= static_cast<int>(buffer_size) )
	    throw std::runtime_error( "readQTMPacket: Buffer not large enough." );

    } while( header->getType() == QTMHeader::PacketEvent );

    // if the response is an error, throw
    if( header->getType() == QTMHeader::PacketError )
	throw std::runtime_error( getBufferString() );

    // check for the right packet type
    if( header->getType() != type )
	return false;

    return true;
}

/** 
 * reads a packet from the stream and interprets it as a string 
 */
bool Driver::readQTMString( std::string& result )
{
    if( readQTMPacket( QTMHeader::PacketCommand ) )
    {
	result = getBufferString();
	return true;
    }
    return false;
}

std::string Driver::getBufferString()
{
    QTMHeader *header = reinterpret_cast<QTMHeader*>(buffer);
    return std::string( reinterpret_cast<char*>(buffer + sizeof(QTMHeader)), header->getPayloadSize() - 1 );
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
