#include <iostream>
#include <qualisys/Driver.hpp>

int main(int argc, char** argv)
{
    if( argc < 2 )
    {
	std::cout << "usage: qualisys_bin <server>" << std::endl;
	exit(0);
    }

    std::string host( argv[1] );

    qualisys::Driver driver;
    std::cout << "Connecting..." << std::endl;
    driver.setReadTimeout( base::Time::fromSeconds( 1.0 ) );
    driver.connect( host );
    std::cout << "done." << std::endl;

    std::cout << "Start Streaming Data..." << std::endl;
    driver.startStreamData();
    for( int i=0; i<10; i++ )
    {
	base::Time ts;
	base::Affine3d transform;
	driver.getTransform( ts, transform, 0 );
    }
    std::cout << "Stop Streaming Data..." << std::endl;
    driver.stopStreamData();
}
