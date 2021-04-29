#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

class cEdge
{
public:
    cEdge()
        : myCost(1)
    {
    }
    int myCost;
};

class cNode
{
public:
    cNode() {}
};

using namespace boost;
typedef boost::adjacency_list<
    boost::listS,
    boost::vecS,
    boost::bidirectionalS,
    cNode,
    cEdge>
    graph_t;

struct sBiLink
{
    int src;
    int dst;
    int dir; // allowed orientations
};

std::vector<std::string> ParseSpaceDelimited(
    const std::string &l)
{
    std::vector<std::string> token;
    std::stringstream sst(l);
    std::string a;
    while (getline(sst, a, ' '))
        token.push_back(a);

    token.erase(
        remove_if(
            token.begin(),
            token.end(),
            [](std::string t) {
                return (t.empty());
            }),
        token.end());

    return token;
}

std::vector<sBiLink> read(
    const std::string &fname,
    std::vector<int> &vTurn,
    sBiLink &start,
    int& goal )
{
    std::ifstream inf(fname);
    if (!inf.is_open())
    {
        std::cout << "cannot open " << fname << "\n";
    }
    std::vector<sBiLink> vBiLink;
    vTurn.clear();
    std::string line;
    int src, dst, dir;
    while (std::getline(inf, line))
    {
        std::cout << line << "\n";
        auto token = ParseSpaceDelimited(line);
        if (!token.size())
            continue;
        switch (token[0][0])
        {
        case 'l':
            if (token.size() != 4)
            {
                std::cout << "bad link: " << line << "\n";
                exit(1);
            }
            sBiLink l;
            l.src = atoi(token[1].c_str());
            l.dst = atoi(token[2].c_str());
            l.dir = atoi(token[3].c_str());
            vBiLink.push_back(l);
            break;

        case 's':
            if (token.size() != 3)
            {
                std::cout << "bad start: " << line << "\n";
                exit(1);
            }
            start.src = atoi(token[1].c_str());
            start.dir = atoi(token[2].c_str());
            break;

        case 'g':
            if (token.size() != 2)
            {
                std::cout << "bad goal: " << line << "\n";
                exit(1);
            }
            goal = atoi(token[1].c_str());
            break;

        case 't':
            if (token.size() != 2)
            {
                std::cout << "bad turning node: " << line << "\n";
                exit(1);
            }
            vTurn.push_back(atoi(token[1].c_str()));
            break;

        default:
            std::cout << "bad row: " << line << "\n";
            break;
        }
    }
    std::cout << vBiLink.size() << " bidirectional links input\n";
    ;
    return vBiLink;
}

/** Split links according to allowed direction of travel
 * @param[in] vBiLink the input links
 * @param[out] vForward links allowing forward travel
 * @param[out] vBack links allowing backward travel
 * @param[out] vTurn modes where robot can turn around
 */
void split(
    const std::vector<sBiLink> &vBiLink,
    std::vector<sBiLink> &vForward,
    std::vector<sBiLink> &vBack,
    std::vector<int> &vTurn)
{

    for (auto &l : vBiLink)
    {
        switch (l.dir)
        {
        case 0:
            vForward.push_back(l);
            vBack.push_back(l);
            break;

        case 1:
            vForward.push_back(l);
            break;

        case 2:
            vBack.push_back(l);
            break;
        }
    }

    std::cout << "\nForward links\n";
    for (auto &l : vForward)
    {
        std::cout
            << std::to_string(l.src)
            << "f - "
            << std::to_string(l.dst)
            << "f\n";
    }
    std::cout << "\nBack links\n";
    for (auto &l : vBack)
    {
        std::cout
            << std::to_string(l.src)
            << "b - "
            << std::to_string(l.dst)
            << "b\n";
    }
    std::cout << "\nTurning Links\n";
    for (auto &n : vTurn)
    {
        std::cout
            << std::to_string(n)
            << "f - "
            << std::to_string(n)
            << "b\n";
    }
}

/// Create boost graph from combination of forward, backward and turning links
graph_t ConstructBoostGraph(
    const std::vector<sBiLink> &vForward,
    const std::vector<sBiLink> &vBack,
    const std::vector<int> &vTurn)
{
    const int dirOffset = 1000;

    graph_t G;

    for (auto &l : vForward)
    {
        boost::add_edge(l.src, l.dst, G);
    }
    for (auto &l : vBack)
    {
        boost::add_edge(dirOffset + l.src, dirOffset + l.dst, G);
    }
    for (int n : vTurn)
    {
        boost::add_edge(n, dirOffset + n, G);
    }

    std::string src, dst;
    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
    {
        if (source(*ei, G) >= dirOffset)
            src = std::to_string(source(*ei, G) - dirOffset) + "b";
        else
            src = std::to_string(source(*ei, G)) + "f";
        if (target(*ei, G) >= dirOffset)
            dst = std::to_string(target(*ei, G) - dirOffset) + "b";
        else
            dst = std::to_string(target(*ei, G)) + "f";
        std::cout << "(" << src
                  << "," << dst << ") ";
    }
    std::cout << "\n";

    return G;
}

void Path(
    graph_t &G,
    sBiLink &start,
    int goal )
{
    std::vector< int > p(num_vertices(G));
    std::vector<int> vDist(num_vertices(G));
    boost::dijkstra_shortest_paths(
        G, start.src,
        weight_map(get(&cEdge::myCost, G))
            .predecessor_map(boost::make_iterator_property_map(
                            p.begin(), get(boost::vertex_index, G)))
            .distance_map(boost::make_iterator_property_map(
                vDist.begin(), get(boost::vertex_index, G))));

    std::vector<int> vpath;
    vpath.push_back( goal );
    while( 1 ) {
        int next = p[ goal ];
        vpath.push_back( next );
        if( next == start.src )
            break;
        goal = next;
    }
    std::reverse(vpath.begin(),vpath.end());
    
    for( auto n : vpath )
        std::cout << n << " -> ";
    std::cout << "\n";
}

main(int argc, char *argv[])
{
    std::cout << "unirobot\n";

    // check command line
    if (argc != 2)
    {
        std::cout << "usage: unirobot <inputfilename>\n";
        exit(0);
    }

    // read input file
    sBiLink start;
    int goal;
    std::vector<int> vTurn;
    auto vBiLink = read(
        argv[1],
        vTurn,
        start,
        goal );

    // split graph
    std::vector<sBiLink> vForward;
    std::vector<sBiLink> vBack;
    split(
        vBiLink,
        vForward,
        vBack,
        vTurn);

    // construct boost graph
    auto G = ConstructBoostGraph(
        vForward,
        vBack,
        vTurn);

    // find path from start to goal
    Path( 
        G,
         start,
         goal );
}