#ifndef QUALISYS_DRIVER_HPP__
#define QUALISYS_DRIVER_HPP__

#include <iostream>
#include <iodrivers_base/Driver.hpp>

#include <base/eigen.h>

namespace qualisys
{

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


class Driver : public iodrivers_base::Driver
{
public: 
    Driver();
    ~Driver();

    /** @brief connect to qualisys server
     *
     * Connect to qualisys server using the given host and base_port. Will throw 
     * if connection attempt is not successful.
     *
     * @param host host of the qualisys server
     * @param base_port the base port number of the server. This is not the actual port
     *			it is connecting to, as this is determined by the endianess of the
     *			connecting machine.
     */
    void connect( const std::string& host, const unsigned int base_port = 22222 );

    /** @brief load marker configuration data from the server
     *
     * loads the marker/body configuration data from the server,
     * and fills the labels array with a list of found labels.
     */
    void loadParameters( std::vector<std::string>& labels );

    /** @brief start streaming marker data from the QTM Server
     *
     * if connected, this will get the QTMServer to start streaming the
     * processed marker data.  The data can be read using the getTransform()
     * method.
     */
    void startStreamData( const std::string& label );

    /** @brief stop streaming of marker data
     */
    void stopStreamData();

    /** @brief read a marker packet
     *
     * Reads a marker packet and store the transformation for the given marker_idx in the given
     * transform reference. The marker/body index is selected in the startStreamData call.
     *
     * @param time [out] timestamp of the read packet
     * @param transform [out] body to world transform of the given marker index
     */
    bool getTransform( base::Time& time, base::Affine3d& transform );

protected:
    bool setQTMVersion( int major, int minor );
    bool readQTMString( std::string& qtmstring );
    std::string getBufferString();
    bool readQTMPacket( QTMHeader::EPacketType type );
    void writeQTMCommand( const std::string& cmd );
    void writeQTMCommand( uint8_t const* buffer, size_t size );

    int extractPacket( uint8_t const* buffer, size_t size) const;

    /// maximum number of markers supported
    static const size_t MAX_MARKERS;

    // internal buffer for storing packets
    uint8_t *buffer;
    size_t buffer_size;

    int body_idx;
};

}

#endif
