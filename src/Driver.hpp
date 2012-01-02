#ifndef QUALISYS_DRIVER_HPP__
#define QUALISYS_DRIVER_HPP__

#include <iostream>
#include <iodrivers_base/Driver.hpp>

#include <base/eigen.h>

namespace qualisys
{

class Driver : public iodrivers_base::Driver
{
public: 
    Driver();

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

    /** @brief start streaming marker data from the QTM Server
     *
     * if connected, this will get the QTMServer to start streaming the
     * processed marker data.  The data can be read using the getTransform()
     * method.
     */
    void startStreamData();

    /** @brief stop streaming of marker data
     */
    void stopStreamData();

    /** @brief read a marker packet
     *
     * Reads a marker packet and store the transformation for the given marker_idx in the given
     * transform reference.
     *
     * @param time [out] timestamp of the read packet
     * @param transform [out] body to world transform of the given marker index
     * @param marker_idx [out] index of the marker for which the transform should be read
     */
    bool getTransform( base::Time& time, base::Affine3d& transform, int marker_idx );

protected:
    bool setQTMVersion( int major, int minor );
    bool readQTMString( std::string& qtmstring );
    void writeQTMCommand( const std::string& cmd );
    void writeQTMCommand( uint8_t const* buffer, size_t size );

    int extractPacket( uint8_t const* buffer, size_t size) const;

    /// maximum number of markers supported
    static const size_t MAX_MARKERS;
};
}

#endif
