#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
using namespace std;

void readNode(vector<glm::vec3>&vertices, map<tuple<int, int, int>, int>&surfaces, map<tuple<int, int, int>, int>& surfacesFinal, map<tuple<int, int>, int>& neighboringEdges)
{
	// .node
	fstream newfile;

	newfile.open("./dragon.node", ios::in); //open a file to perform read operation using file object
	if (newfile.is_open()) {   //checking whether the file is open
		string tp;
		int firstLine = 0;
		int nodeSize = 1;
		while (getline(newfile, tp) && nodeSize != 0) { //read data from file object and put it into string.
		   // cout << tp << "\n"; //print the data of the string

			if (firstLine == 0)
			{
				string num = "";
				vector<double> info;
				for (int i = 0; i < tp.length(); i++)
				{
					if (tp[i] != ' ')
					{
						char c = tp[i];
						num += c;
					}
					else
					{
						if (num != "")
						{
							info.push_back(atof(num.c_str()));
							num = "";
						}
					}
				}
				nodeSize = info[0];
				firstLine++;
				continue;
			}

			string num = "";
			vector<double> vertex;
			for (int i = 0; i < tp.length(); i++)
			{
				if (tp[i] != ' ')
				{
					char c = tp[i];
					num += c;
				}
				else
				{
					if (num != "")
					{
						vertex.push_back(atof(num.c_str()));
						num = "";
					}
				}
			}

			if (num != "")
			{
				vertex.push_back(atof(num.c_str()));
				num = "";
			}

			vertices.push_back(glm::vec3(vertex[1], vertex[2], vertex[3]));
			nodeSize--;
		}
		newfile.close(); //close the file object.

		std::cout << vertices.size() << std::endl;
	}

	// .ele
	vector<vector<double>> edges;

	newfile.open("./dragon.ele", ios::in); //open a file to perform read operation using file object
	if (newfile.is_open()) {   //checking whether the file is open
		string tp;
		int firstLine = 0;
		int trianglesSize = 1;
		while (getline(newfile, tp) && trianglesSize != 0) { //read data from file object and put it into string.

		  // first line
			if (firstLine == 0)
			{
				string num = "";
				vector<double> info;
				for (int i = 0; i < tp.length(); i++)
				{
					if (tp[i] != ' ')
					{
						char c = tp[i];
						num += c;
					}
					else
					{
						if (num != "")
						{
							info.push_back(atof(num.c_str()));
							num = "";
						}
					}
				}
				trianglesSize = info[0];
				firstLine++;
				continue;
			}

			// others
			string num = "";
			vector<double> edge;
			for (int i = 0; i < tp.length(); i++)
			{
				if (tp[i] != ' ')
				{
					char c = tp[i];
					num += c;
				}
				else
				{
					if (num != "")
					{
						edge.push_back(atof(num.c_str()));
						num = "";
					}
				}
			}

			if (num != "")
			{
				edge.push_back(atof(num.c_str()));
				num = "";
			}

			tuple<int, int, int> key(edge[1], edge[3], edge[2]);
			if (surfaces.find(key) == surfaces.end())
			{
				surfaces[key] = 0;
			}
			else
			{
				surfaces[key]++;
			}

			tuple<int, int, int> key2(edge[4], edge[2], edge[3]);
			if (surfaces.find(key2) == surfaces.end())
			{
				surfaces[key2] = 0;
			}
			else
			{
				surfaces[key2]++;
			}

			tuple<int, int, int> key3(edge[4], edge[3], edge[1]);
			if (surfaces.find(key3) == surfaces.end())
			{
				surfaces[key3] = 0;
			}
			else
			{
				surfaces[key3]++;
			}

			tuple<int, int, int> key4(edge[4], edge[1], edge[2]);
			if (surfaces.find(key4) == surfaces.end())
			{
				surfaces[key4] = 0;
			}
			else
			{
				surfaces[key4]++;
			}

			//		edges.push_back(edge);
			trianglesSize--;
		}

		map<tuple<int, int, int>, int>::iterator iter;
		for (iter = surfaces.begin(); iter != surfaces.end(); iter++)
		{
			int i = iter->second;
			if (i == 0)
				surfacesFinal[iter->first] = 0;
		}

		for (iter = surfacesFinal.begin(); iter != surfacesFinal.end(); iter++)
		{
			tuple<int,int,int> edge = iter->first;

			// save edge
			tuple<int, int> edges1(get<0>(edge), get<1>(edge));
			tuple<int, int> edges2(get<1>(edge), get<0>(edge));
			if (neighboringEdges.find(edges1) == neighboringEdges.end() && neighboringEdges.find(edges2) == neighboringEdges.end())
			{
				neighboringEdges[edges1] = 0;
			}
			tuple<int, int> edges3(get<0>(edge), get<2>(edge));
			tuple<int, int> edges4(get<2>(edge), get<0>(edge));
			if (neighboringEdges.find(edges3) == neighboringEdges.end() && neighboringEdges.find(edges4) == neighboringEdges.end())
			{
				neighboringEdges[edges3] = 0;
			}
			tuple<int, int> edges5(get<1>(edge), get<2>(edge));
			tuple<int, int> edges6(get<2>(edge), get<1>(edge));
			if (neighboringEdges.find(edges5) == neighboringEdges.end() && neighboringEdges.find(edges6) == neighboringEdges.end())
			{
				neighboringEdges[edges5] = 0;
			}
		}

		newfile.close(); //close the file object.
	}
}