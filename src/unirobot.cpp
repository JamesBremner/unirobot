#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

struct sBiLink
{
    int src;
    int dst;
    int dir;        // allowed orientations
};

std::vector< std::string > ParseSpaceDelimited(
    const std::string& l )
{
    std::vector< std::string > token;
    std::stringstream sst(l);
    std::string a;
    while( getline( sst, a, ' ' ) )
        token.push_back( a );

    token.erase(
        remove_if(
            token.begin(),
            token.end(),
            [] ( std::string t )
    {
        return( t.empty() );
    } ),
    token.end() );

    return token;
}

std::vector< sBiLink > read( 
    const std::string& fname,
    std::vector< int>& vTurn )
{
    std::ifstream inf( fname );
    if( ! inf.is_open() )
    {
        std::cout << "cannot open " << fname << "\n";
    }
    std::vector< sBiLink > vBiLink;
    vTurn.clear();
    std::string line;
    int src, dst, dir;
    while( std::getline(inf,line))
    {
        std::cout << line << "\n";
        auto token = ParseSpaceDelimited( line );
        if( ! token.size() )
            continue;
        switch( token[0][0] )
        {
            case 'l':
            if( token.size() != 4 )
            {
                std::cout << "bad link: " << line << "\n";
                exit(1);
            }
            sBiLink l;
            l.src = atoi( token[1].c_str() );
            l.dst = atoi( token[2].c_str() );
            l.dir = atoi( token[3].c_str() );
            vBiLink.push_back( l );
            break;

            case 's':
            if( token.size() != 3 )
            {
                std::cout << "bad start: " << line << "\n";
                exit(1);
            }      
            src = atoi( token[1].c_str() );
            dir = atoi( token[2].c_str() );   
            break;

            case 'g':
            if( token.size() != 2 )
            {
                std::cout << "bad start: " << line << "\n";
                exit(1);
            }    
            dir = atoi( token[1].c_str() ); 
            break; 

            case 't':
            if( token.size() != 2 )
            {
                std::cout << "bad turning node: " << line << "\n";
                exit(1);
            }    
            vTurn.push_back( atoi( token[1].c_str() ) );
            break;

            default:
            std::cout << "bad row: " << line << "\n";
            break;
        }
    }
    std::cout << vBiLink.size() << " bidirectional links input\n";;
    return vBiLink;
}

void split( 
    std::vector< sBiLink >& vBiLink,
    std::vector< int >& vTurn )
{
    std::vector< sBiLink > vForward;
    std::vector< sBiLink > vBack;

    for( auto& l : vBiLink )
    {
        switch( l.dir )
        {
            case 0:
            vForward.push_back( l );
            vBack.push_back( l );
            break;

            case 1:
            vForward.push_back( l );
            break;

            case 2:
            vBack.push_back( l );
            break;   
        }
    }

    std::cout << "\nForward links\n";
    for( auto& l : vForward )
    {
        std:: cout
            << std::to_string( l.src ) 
            << "f - "
            << std::to_string( l.dst ) 
            << "f\n";
    }
    std::cout << "\nBack links\n";
    for( auto& l : vBack )
    {
        std:: cout
            << std::to_string( l.src ) 
            << "b - "
            << std::to_string( l.dst ) 
            << "b\n";
    }
    std::cout << "\nTurning Links\n";
    for( auto& n : vTurn )
    {
        std:: cout
            << std::to_string( n ) 
            << "f - "
            << std::to_string( n ) 
            << "b\n";
    }
}
main( int argc, char* argv[] )
{
    std::cout << "unirobot\n";

    // check command line
    if( argc != 2 )
    {
        std::cout << "usage: unirobot <inputfilename>\n";
        exit(0);
    }

    // read input file
    std::vector< int> vTurn;
    auto vBiLink = read( argv[1], vTurn );

    // split graph
    split( vBiLink, vTurn );
}