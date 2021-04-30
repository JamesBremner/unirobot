#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

/// edge properties
class cEdge
{
public:
    cEdge()
        : myCost(1)
    {
    }
    int myCost;
};

/// node properties
class cNode
{
public:
    cNode(const std::string &name)
        : myName(name)
    {
    }
    cNode()
        : myName("???")
    {
    }
    std::string myName;
};

/// declaration of boost graph configuration
using namespace boost;
typedef boost::adjacency_list<
    boost::listS,
    boost::vecS,
    boost::directedS,
    cNode,
    cEdge>
    graph_t;

/// declaration of structure holding link input data
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

/// read input
std::vector<sBiLink> read(
    const std::string &fname,
    std::vector<int> &vTurn,
    sBiLink &start,
    int &goal)
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
            if (token[2] == "b")
                start.dir = 2;
            else
                start.dir = 1;
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
/** Find node by name
 * @return index of node found
 * 
 * If node by same bname already exists, return its index
 * otherwise add a new node and return its index
 */
int findoradd(graph_t &G, const std::string &name)
{
    for (int n = 0; n < num_vertices(G); n++)
    {
        if (G[n].myName == name)
        {
            return n;
        }
    }
    return add_vertex(name, G);
}

/// Create boost graph from combination of forward, backward and turning links
graph_t ConstructBoostGraph(
    const std::vector<sBiLink> &vForward,
    const std::vector<sBiLink> &vBack,
    const std::vector<int> &vTurn)
{
    graph_t G;

    for (auto &l : vForward)
    {
        auto s = findoradd(G, std::to_string(l.src) + "f");
        auto d = findoradd(G, std::to_string(l.dst) + "f");
        boost::add_edge(s, d, G);
        boost::add_edge(d, s, G);
    }
    for (auto &l : vBack)
    {
        auto s = findoradd(G, std::to_string(l.src) + "b");
        auto d = findoradd(G, std::to_string(l.dst) + "b");
        boost::add_edge(s, d, G);
        boost::add_edge(d, s, G);
    }
    for (int n : vTurn)
    {
        // add edge begining and ending on node
        // which allows the robot to turn around
        // at zero cost
        auto s = findoradd(G, std::to_string(n) + "f");
        auto d = findoradd(G, std::to_string(n) + "b");
        auto e = boost::add_edge(s, d, G).first;
        G[e].myCost = 0;
        e = boost::add_edge(d, s, G).first;
        G[e].myCost = 0;
    }

    std::cout << "\nCombined graph links\n";
    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
    {
        std::cout
            << "("
            << G[source(*ei, G)].myName << ","
            << G[target(*ei, G)].myName
            << ") ";
    }
    std::cout << "\n";

    return G;
}
/** Find path
 * @param[in] G the graph
 * @param[in] start starting node info
 * @param[in] goal index of node aiming for
 * @return vector of node indices visited on path
 */
std::vector<int> Path(
    graph_t &G,
    sBiLink &start,
    int goal)
{
    // starting node in boost graph
    int startNode = -1;
    std::string startNodeName = std::to_string(start.src);
    if (start.dir == 1)
        startNodeName += "f";
    else
        startNodeName += "b";
    for (int n = 0; n < num_vertices(G); n++)
    {
        if (G[n].myName == startNodeName)
        {
            startNode = n;
            break;
        }
    }
    if (startNode < 0)
        throw std::runtime_error("Bad path start");
    //std::cout << "start " << startNodeName << " " << startNode << "\n";

    // goal nodes in boost graph
    int goalf, goalb;
    goalf = goalb = -1;
    std::string sgf = std::to_string(goal) + "f";
    std::string sgb = std::to_string(goal) + "b";
    for (int n = 0; n < num_vertices(G); n++)
    {
        if (G[n].myName == sgf)
            goalf = n;
        if (G[n].myName == sgb)
            goalb = n;
    }
    if (goalf < 0 && goalb < 0)
        throw std::runtime_error("Bad path goal");

    // run dijkstra algorithm
    std::vector<int> p(num_vertices(G));
    std::vector<int> vDist(num_vertices(G));
    boost::dijkstra_shortest_paths(
        G, startNode,
        weight_map(get(&cEdge::myCost, G))
            .predecessor_map(boost::make_iterator_property_map(
                p.begin(), get(boost::vertex_index, G)))
            .distance_map(boost::make_iterator_property_map(
                vDist.begin(), get(boost::vertex_index, G))));

    // choose closest possible goal
    int goalnode = goalf;
    if (goalf < 0)
        goalnode = goalb;
    else
    {
        if (vDist[goalb] < vDist[goalf] )
            goalnode = goalb;
    }

    // pick out path, starting at goal and finishing at start
    std::vector<int> vpath;
    vpath.push_back(goalnode);
    int prev = goalnode;
    while (1)
    {
        //std::cout << prev << " " << p[prev] << ", ";
        int next = p[prev];
        vpath.push_back(next);
        if (next == startNode)
            break;
        prev = next;
    }

    // reverse so path goes from start to goal
    std::reverse(vpath.begin(), vpath.end());

    return vpath;
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
        goal);

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
    auto vpath = Path(
        G,
        start,
        goal);

    // display path found
    std::cout << "\nPath: ";
    for (auto n : vpath)
        std::cout << G[n].myName << " -> ";
    std::cout << "\n";
}