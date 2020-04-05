#pragma once
#include "ofMain.h"

class tb
{
public:
	struct bounds
	{
		bounds() { min = 0; max = 1.0f; };
		bounds(float minimum, float maximum)
		{
			min = minimum;
			max = maximum;
		}
		float min;
		float max;
	};

	// uses normalized texcoords
	static ofMesh gridMesh(int w, int h, float scale, bool center)
	{
		ofMesh mesh;
		mesh.setMode(OF_PRIMITIVE_TRIANGLES);

		glm::vec3 size = glm::vec3(w, 0, h) * scale;

		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				float x_mesh = x * scale;
				float y_mesh = y * scale;

				mesh.addVertex(glm::vec3(x_mesh, 0, y_mesh) - (center ? size/2.0f:glm::vec3(0)));
				mesh.addNormal(glm::vec3(0, 1, 0));
				mesh.addTexCoord(glm::vec2((float)x / (float)w, (float)y / (float)h));
			}
		}

		for (int y = 0; y < h - 1; y++) {
			for (int x = 0; x < w - 1; x++) {
				//if (x % 2 == 0) {
				mesh.addIndex(x + y * w);				// a
				mesh.addIndex((x + 1) + y * w);			// b
				mesh.addIndex(x + (y + 1) * w);			// d

				mesh.addIndex((x + 1) + y * w);			// b
				mesh.addIndex((x + 1) + (y + 1) * w);		// c
				mesh.addIndex(x + (y + 1) * w);			// d
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

	static ofMesh tb::loadObj(string filename)
	{
		ofMesh m;
		bool smooth = false;

		vector<glm::vec3> v;
		vector<glm::vec3> vn;
		vector<glm::vec2> vt;
		ofFile f(filename);

		while (!f.eof()) {
			string c;
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
					string l;
					getline(f, l);
					replace(l.begin(), l.end(), '/', ' ');
					istringstream ls(l);
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

	static void tb::flipNormals(ofMesh& mesh)
	{
		for (int i = 0; i < mesh.getNumNormals(); i++) {
			mesh.getNormals()[i] = mesh.getNormals()[i] * -1.0f;
		}
	}

	static void tb::drawNormals(ofMesh& mesh, float len)
	{
		for (int i = 0; i < mesh.getNumNormals(); i++) {
			glm::vec3 normal = glm::normalize(mesh.getNormals()[i]);
			ofDrawLine(mesh.getVertices()[i], mesh.getVertices()[i] + normal * len);
		}
	}

	static ofMesh tb::rectMesh(float x, float y, float w, float h, const ofTexture& tex)
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

	static ofMesh tb::rectMesh(ofRectangle rect, const ofTexture& tex)
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

	static ofMesh tb::rectMesh(float x, float y, float w, float h, bool normalized)
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

	static std::string tb::saveTexture(const ofTexture& tex, std::string id)
	{
		ofImage img;
		ofPixels pixels;

		std::string dirname = "output";
		std::string ext = ".png";
		char path[128];

		tex.readToPixels(pixels);
		img.setFromPixels(pixels);

		sprintf(path, "%s/%s_%s%s", dirname.c_str(), id.c_str(), ofGetTimestampString("%Y-%m-%d-%H%M%S%i").c_str(), ext.c_str());
		img.save(path, ofImageQualityType::OF_IMAGE_QUALITY_BEST);

		return path;
	}

	static glm::vec3 getMouse()
	{
		return glm::vec3(ofGetMouseX(), ofGetMouseY(), 0);
	}

	static glm::vec3 lim(const glm::vec3& v, float min, float max)
	{
		float len = length(v);
		if (len > max) {
			return normalize(v) * max;
		}
		else if (len < min) {
			return normalize(v) * min;
		}
		return v;
	}

	static glm::quat quatDiff(glm::vec3 u, glm::vec3 v)
	{
		float norm_u_norm_v = glm::sqrt(glm::dot(u, u) * glm::dot(v, v));
		float real_part = norm_u_norm_v + glm::dot(u, v);
		glm::vec3 w;

		if (real_part < 1.e-6f * norm_u_norm_v) {
			real_part = 0.0f;
			w = glm::abs(u.x) > glm::abs(u.z) ? glm::vec3(-u.y, u.x, 0.f) : glm::vec3(0.f, -u.z, u.y);
		}
		else {
			w = glm::cross(u, v);
		}
		return glm::normalize(glm::quat(real_part, w.x, w.y, w.z));
	}

	// maxAngle in radians
	static glm::quat rotateTowards(glm::quat q0, glm::quat q1, float mix)
	{
		float cosTheta = glm::dot(q0, q1);
		if (cosTheta > 0.9999f) return q1;

		// Avoid taking the long path around the sphere
		if (cosTheta < 0) {
			q0 = q0 * -1.0f;
			cosTheta *= -1.0f;
		}
		float angle = glm::acos(cosTheta);
		if (angle < 0.001f) return q1;

		//float mixFactor = mix / angle;
		float mixFactor = mix;

		return glm::slerp(q0, q1, mixFactor);
	}

	// Returns a quaternion that will make your object looking towards 'direction'.
	// Similar to RotationBetweenVectors, but also controls the vertical orientation.
	// This assumes that at rest, the object faces +Z.
	// Beware, the first parameter is a direction, not the target point !
	static glm::quat lookAt(glm::vec3 direction, glm::vec3 forward, glm::vec3 up)
	{
		if (glm::length2(direction) < 0.0001f)
			return glm::quat();

		// Recompute desiredUp so that it's perpendicular to the direction
		// You can skip that part if you really want to force desiredUp
		glm::vec3 right = glm::cross(direction, up);
		up = glm::cross(right, direction);

		// Find the rotation between the front of the object (that we assume towards +Z,
		// but this depends on your model) and the desired direction
		glm::quat rot1 = tb::quatDiff(forward, direction);

		// Because of the 1rst rotation, the up is probably completely screwed up. 
		// Find the rotation between the "up" of the rotated object, and the desired up
		glm::vec3 newUp = rot1 * glm::vec3(0.0f, 0.0f, 1.0f);
		glm::quat rot2 = tb::quatDiff(newUp, up);

		return rot2 * rot1;
	}
};
