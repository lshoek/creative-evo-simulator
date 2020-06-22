#include "ofVectorMath.h"
#include "ofGraphics.h"
#include "ofPoint.h"
#include "ofMesh.h"
#include "ofTexture.h"

class MeshUtils
{
public:
	// uses normalized texcoords
	static ofMesh gridMesh(int w, int h, float scale, bool bCenter)
	{
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_TRIANGLES);

		glm::vec3 correctCenter = bCenter ? (scale / 2.0f) * glm::vec3(1, 0, 1) : glm::vec3(0);

		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				float x_mesh = x / float(w - 1) * scale;
				float y_mesh = y / float(h - 1) * scale;

				mesh.addVertex(glm::vec3(x_mesh, 0, y_mesh) - correctCenter);
				mesh.addNormal(glm::vec3(0, 1.0f, 0));
				mesh.addTexCoord(glm::vec2(x / float(w - 1), y / float(h - 1)));
			}
		}

		for (int y = 0; y < h - 1; y++) {
			for (int x = 0; x < w - 1; x++) {

				//if (x % 2 == 0) {
				mesh.addIndex(x + y * w);				// a
				mesh.addIndex((x + 1) + (y + 1) * w);	// c
				mesh.addIndex(x + (y + 1) * w);			// d

				mesh.addIndex(x + y * w);				// a
				mesh.addIndex((x + 1) + y * w);			// b
				mesh.addIndex((x + 1) + (y + 1) * w);	// c

			//}
			//else {
			//	mesh.addIndex((x + 1) + y * w);			// b
			//	mesh.addIndex(x + y * w);				// a
			//	mesh.addIndex((x + 1) + (y + 1)*w);		// c

			//	mesh.addIndex(x + y * w);				// a
			//	mesh.addIndex(x + (y + 1)*w);			// d
			//	mesh.addIndex((x + 1) + (y + 1)*w);		// c
			//}

			}
		}
		return mesh;
	}

	static void setMeshNormals(ofMesh& mesh, bool bNormalize)
	{
		//The number of the vertices
		int nV = mesh.getNumVertices();

		//The number of the triangles
		int nT = mesh.getNumIndices() / 3;

		std::vector<glm::vec3> norm;
		norm.reserve(nV);

		for (int i = 0; i < nV; i++) {
			norm.push_back(glm::vec3(0.0f));
		}

		//Scan all the triangles. For each triangle add its
		//normal to norm's vectors of triangle's vertices
		for (int t = 0; t < nT; t++) {

			//Get indices of the triangle t
			int i1 = mesh.getIndex(3 * t);
			int i2 = mesh.getIndex(3 * t + 1);
			int i3 = mesh.getIndex(3 * t + 2);

			//Get vertices of the triangle
			const glm::vec3& v1 = mesh.getVertex(i1);
			const glm::vec3& v2 = mesh.getVertex(i2);
			const glm::vec3& v3 = mesh.getVertex(i3);

			glm::vec3 d1 = v2 - v1;
			glm::vec3 d2 = v3 - v1;

			//Compute the triangle's normal
			glm::vec3 dir = glm::normalize(glm::cross(d1, d2));

			//Accumulate it to norm array for i1, i2, i3
			norm[i1] += dir;
			norm[i2] += dir;
			norm[i3] += dir;
		}

		if (bNormalize) {
			for (int i = 0; i < nV; i++) {
				norm[i] = glm::normalize(norm[i]);
			}
		}
		mesh.clearNormals();
		mesh.addNormals(norm);
	}

	static void flipNormals(ofMesh& mesh)
	{
		for (int i = 0; i < mesh.getNumNormals(); i++) {
			mesh.getNormals()[i] = mesh.getNormals()[i] * -1.0f;
		}
	}

	static void drawNormals(ofMesh& mesh, float len)
	{
		for (int i = 0; i < mesh.getNumNormals(); i++) {
			glm::vec3 normal = glm::normalize(mesh.getNormals()[i]);
			ofDrawLine(mesh.getVertices()[i], mesh.getVertices()[i] + normal * len);
		}
	}

	static ofMesh rectMesh(float x, float y, float w, float h, const ofTexture& tex)
	{
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

		mesh.addVertex(ofPoint(x, y));
		mesh.addTexCoord(tex.getCoordFromPercent(0, 0));

		mesh.addVertex(ofPoint(x + w, y));
		mesh.addTexCoord(tex.getCoordFromPercent(1, 0));

		mesh.addVertex(ofPoint(x, y + h));
		mesh.addTexCoord(tex.getCoordFromPercent(0, 1));

		mesh.addVertex(ofPoint(x + w, y + h));
		mesh.addTexCoord(tex.getCoordFromPercent(1, 1));

		return mesh;
	}

	static ofMesh rectMesh(ofRectangle rect, const ofTexture& tex)
	{
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

		mesh.addVertex(ofPoint(rect.getTopLeft().x, rect.getTopLeft().y));
		mesh.addTexCoord(tex.getCoordFromPercent(0, 0));

		mesh.addVertex(ofPoint(rect.getTopRight().x, rect.getTopRight().y));
		mesh.addTexCoord(tex.getCoordFromPercent(1, 0));

		mesh.addVertex(ofPoint(rect.getBottomLeft().x, rect.getBottomLeft().y));
		mesh.addTexCoord(tex.getCoordFromPercent(0, 1));

		mesh.addVertex(ofPoint(rect.getBottomRight().x, rect.getBottomRight().y));
		mesh.addTexCoord(tex.getCoordFromPercent(1, 1));

		return mesh;
	}

	static ofMesh rectMesh(float x, float y, float w, float h, bool normalized)
	{
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

		mesh.addVertex(ofPoint(x, y));
		mesh.addTexCoord(glm::vec2(0, 0));

		mesh.addVertex(ofPoint(x + w, y));
		mesh.addTexCoord(glm::vec2(normalized ? 1 : w, 0));

		mesh.addVertex(ofPoint(x, y + h));
		mesh.addTexCoord(glm::vec2(0, normalized ? 1 : h));

		mesh.addVertex(ofPoint(x + w, y + h));
		mesh.addTexCoord(glm::vec2(normalized ? 1 : w, normalized ? 1 : h));

		return mesh;
	}


	static ofMesh loadObj(std::string filename)
	{
		ofMesh m;
		bool smooth = false;

		std::vector<glm::vec3> v;
		std::vector<glm::vec3> vn;
		std::vector<glm::vec2> vt;
		ofFile f(filename);

		while (!f.eof()) {
			std::string c;
			f >> c;
			if (c.size()) {
				if (c == "v") {
					float x, y, z;
					f >> x >> y >> z;
					if (smooth) {
						m.addVertex(glm::vec3(x, y, z));
					}
					else {
						v.push_back(glm::vec3(x, y, z));
					}
				}
				else if (c == "vt") {
					float u, v;
					f >> u >> v;
					if (!smooth) {
						vt.push_back(glm::vec2(u, v));
					}
				}
				else if (c == "vn") {
					float x, y, z;
					f >> x >> y >> z;
					vn.push_back(glm::vec3(x, y, z));
				}
				else if (c == "f") {
					std::string l;
					getline(f, l);
					replace(l.begin(), l.end(), '/', ' ');
					std::istringstream ls(l);
					int vi1, vti1, vni1, vi2, vti2, vni2, vi3, vti3, vni3;
					ls >> vi1 >> vti1 >> vni1 >> vi2 >> vti2 >> vni2 >> vi3 >> vti3 >> vni3;
					if (smooth) {
						m.addIndex(vi1 - 1);
						m.addIndex(vi2 - 1);
						m.addIndex(vi3 - 1);
					}
					else {
						m.addVertex(v[vi1 - 1]);
						m.addVertex(v[vi2 - 1]);
						m.addVertex(v[vi3 - 1]);
						m.addTexCoord(vt[vti1 - 1]);
						m.addTexCoord(vt[vti2 - 1]);
						m.addTexCoord(vt[vti3 - 1]);
						m.addNormal(vn[vni1 - 1]);
						m.addNormal(vn[vni2 - 1]);
						m.addNormal(vn[vni3 - 1]);
					}
					if (ls.peek() == ' ') {
						int vi4, vti4;
						ls >> vi4 >> vti4;
						if (smooth) {
							m.addIndex(vi1 - 1);
							m.addIndex(vi3 - 1);
							m.addIndex(vi4 - 1);
						}
						else {
							m.addVertex(v[vi1 - 1]);
							m.addVertex(v[vi3 - 1]);
							m.addVertex(v[vi4 - 1]);
							m.addTexCoord(vt[vti1 - 1]);
							m.addTexCoord(vt[vti2 - 1]);
							m.addTexCoord(vt[vti3 - 1]);
						}
					}
				}
			}
		}
		return m;
	}
};
