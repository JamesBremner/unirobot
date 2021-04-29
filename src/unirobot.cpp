#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

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

void read( const std::string& fname )
{
    std::ifstream inf( fname );
    if( ! inf.is_open() )
    {
        std::cout << "cannot open " << fname << "\n";
    }
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
            src = atoi( token[1].c_str() );
            dst = atoi( token[2].c_str() );
            dir = atoi( token[3].c_str() );
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
            src = atoi( token[1].c_str() );
            break;

            default:
            std::cout << "bad row: " << line << "\n";
            break;
        }
    }
}
main( int argc, char* argv[] )
{
    std::cout << "unirobot\n";

    if( argc != 2 )
    {
        std::cout << "usage: unirobot <inputfilename>\n";
        exit(0);
    }

    read(argv[1]);
}