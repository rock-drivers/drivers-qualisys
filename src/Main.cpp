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

    std::cout << "Load parameters..." << std::endl;
    std::vector<std::string> labels;
    driver.loadParameters( labels );
    for( size_t i=0; i<labels.size(); i++ )
    {
	std::cout << i << " " << labels[i] << std::endl;
    }

    if( labels.size() > 0 )
    {
	std::cout << "Start Streaming Data..." << std::endl;
	driver.startStreamData( labels[0] );
	for( int i=0; i<10; i++ )
	{
	    base::Time ts;
	    base::Affine3d transform;
	    driver.getTransform( ts, transform );
	}
	std::cout << "Stop Streaming Data..." << std::endl;
	driver.stopStreamData();
    }
    else
    {
	std::cout << "Server has not provided any bodies" << std::endl;
    }
}
