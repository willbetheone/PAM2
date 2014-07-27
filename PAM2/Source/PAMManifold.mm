//
//  PAMManifold.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PAMManifold.h"
#include "Vec4uc.h"
#include "RALogManager.h"
#include <OpenGLES/ES2/glext.h>
#include <GLKit/GLKMath.h>
#include <limits.h>
#include "../HMesh/obj_load.h"
#include <map>
#include "RAPolylineUtilities.h"
#include "Quatf.h"
#include "PAMUtilities.h"

#define kCENTROID_STEP 0.025f

namespace PAMMesh
{
    using namespace HMesh;
    using namespace RAEngine;
    using namespace CGLA;
    using namespace std;
    using namespace Geometry;

    
#pragma mark - CONSTRUCTOR/DESTRUCTOR
    PAMManifold::PAMManifold() : HMesh::Manifold() {}
    PAMManifold::~PAMManifold()
    {
        delete kdTree;
    }

#pragma mark - INHERITED VIRTUAL FUNCTTIONS
    int PAMManifold::loadObjFile(const char *path)
    {
        if (!HMesh::obj_load(path, *this)) {
            RA_LOG_ERROR("Failed to load obj file %s", path);
            return 0;
        }
        return 1;
    }
    
    RAEngine::Bounds PAMManifold::getBoundingBox() const
    {
        Vec3d pmin;
        Vec3d pmax;
        HMesh::bbox(*this, pmin, pmax);
        
        Vec3d mid = pmax - pmin;
        float radius = 0.5*mid.length();
        Vec3d center =  pmin + radius*normalize(mid);
        
        Bounds bnds = {Vec3(pmin), Vec3(pmax), Vec3(center), radius};
        return bnds;
    }
        
#pragma mark - PUBLIC FUNCTIONS
    
    void PAMManifold::setupShaders()
    {
        std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"].UTF8String);
        std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"].UTF8String);
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vShader_Cplus, fShader_Cplus);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_NORMAL] = drawShaderProgram->getAttributeLocation("aNormal");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
        uniforms[UNIFORM_NORMAL_MATRIX] = drawShaderProgram->getUniformLocation("uNormalMatrix");
    }
   
    
    void PAMManifold::bufferVertexDataToGPU()
    {
        Vec3f* vertexPositions;
        Vec3f* vertexNormals;
        Vec4uc* vertexColors;
        std::vector<unsigned int>* indicies;
        CGLA::Vec4uc* wireframeColor;
        std::vector<unsigned int>* wireframeIndicies;
        
        getVertexData(vertexPositions,
                      vertexNormals,
                      vertexColors,
                      indicies,
                      wireframeColor,
                      wireframeIndicies);

        //MESH
        assert(no_vertices() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numVerticies = (GLsizei)no_vertices();
        
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   vertexPositions,
                                                   GL_DYNAMIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
        normalDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                 numVerticies,
                                                 vertexNormals,
                                                 GL_STATIC_DRAW,
                                                 GL_ARRAY_BUFFER);
        normalDataBuffer->enableAttribute(attrib[ATTRIB_NORMAL]);
        
        colorDataBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                numVerticies,
                                                vertexColors,
                                                GL_STATIC_DRAW,
                                                GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);

        assert(indicies->size() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numIndicies = (GLsizei)indicies->size();
        
        indexDataBuffer = new RAES2VertexBuffer(sizeof(unsigned int),
                                                numIndicies,
                                                indicies->data(),
                                                GL_STATIC_DRAW,
                                                GL_ELEMENT_ARRAY_BUFFER);
        
        //WIREFRAME
        wireframeColorBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                     numVerticies,
                                                     wireframeColor,
                                                     GL_STATIC_DRAW,
                                                     GL_ARRAY_BUFFER);
        wireframeColorBuffer->enableAttribute(attrib[ATTRIB_COLOR]);
        
        assert(wireframeIndicies->size() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numWireframeIndicies = (GLsizei)wireframeIndicies->size();
        
        wireframeIndexBuffer = new RAES2VertexBuffer(sizeof(unsigned int),
                                                     numWireframeIndicies,
                                                     wireframeIndicies->data(),
                                                     GL_STATIC_DRAW,
                                                     GL_ELEMENT_ARRAY_BUFFER);
        delete[] vertexPositions;
        delete[] vertexNormals;
        delete[] vertexColors;
        delete[] wireframeColor;
        delete indicies;
        delete wireframeIndicies;
    }
    
//    void PAMManifold::normalizeVertexCoordinates()
//    {
//        //Calculate Bounding Box
//        Vec3d pmin;
//        Vec3d pmax;
//        HMesh::bbox(*this, pmin, pmax);
//        
//        Vec3d midV = 0.5 * (pmax - pmin);
//        float rad = midV.length();
//        
//        for (VertexID vID: this->vertices()) {
//            Vec3d pos = this->pos(vID);
//            Vec3d newPos = (pos - pmin - midV) / rad;
//            this->pos(vID) = newPos;
//        }
//    }
    
    void PAMManifold::getVertexData(CGLA::Vec3f*& vertexPositions,
                                    CGLA::Vec3f*& vertexNormals,
                                    CGLA::Vec4uc*& vertexColors,
                                    std::vector<unsigned int>*& indicies,
                                    CGLA::Vec4uc*& wireframeColor,
                                    std::vector<unsigned int>*& wireframeIndicies) const
    {
        Vec4uc color(200,200,200,255);
        Vec4uc wcolor(180,180,180,255);
        
        vertexPositions = new Vec3f[no_vertices()];
        vertexNormals = new Vec3f[no_vertices()];
        vertexColors = new Vec4uc[no_vertices()];
        indicies = new vector<unsigned int>();
        
        wireframeColor = new Vec4uc[no_vertices()];
        wireframeIndicies = new vector<unsigned int>();
        
        // when interating through faces we need to map VertexID to index
        std::map<VertexID, int>* vertexIDtoIndex = new std::map<VertexID, int>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
            vertexPositions[i] = posf(*vid);
            vertexNormals[i] = HMesh::normalf(*this, *vid);
            vertexColors[i] = color;
            wireframeColor[i] = wcolor;
            (*vertexIDtoIndex)[*vid] = i;
        }
        
        for (FaceIDIterator fid = faces_begin(); fid != faces_end(); ++fid) {
            int vertexNum = 0;
            unsigned int facet[4];
            
            //iterate over every vertex of the face
            for (Walker w = walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
//                unsigned int index = vID.index;
                unsigned int index = (*vertexIDtoIndex)[vID];
                assert(index < no_vertices());
                facet[vertexNum] = index;
                vertexNum++;
                
                if (vertexNum == 4)
                {
                    //Create a second triangle
                    indicies->push_back(facet[0]);
                    indicies->push_back(facet[2]);
                }
                indicies->push_back(index);
            }
            
            //add wireframe data
            if (vertexNum == 3 || vertexNum == 4)
            {
                wireframeIndicies->push_back(facet[0]);
                wireframeIndicies->push_back(facet[1]);
                wireframeIndicies->push_back(facet[1]);
                wireframeIndicies->push_back(facet[2]);
                
                if (vertexNum == 3)
                {
                    wireframeIndicies->push_back(facet[2]);
                    wireframeIndicies->push_back(facet[0]);
                }
                else if (vertexNum == 4)
                {
                    wireframeIndicies->push_back(facet[2]);
                    wireframeIndicies->push_back(facet[3]);
                    wireframeIndicies->push_back(facet[3]);
                    wireframeIndicies->push_back(facet[0]);
                }
            }
        }
        
        delete vertexIDtoIndex;
    }
    
    bool PAMManifold::normal(const CGLA::Vec3f& point, CGLA::Vec3f& norm)
    {
        VertexID vid;
        if (closestVertexID_3D(point, vid)){
            norm = normalf(*this, vid);
            return true;
        }
        return false;
    }
    
    bool PAMManifold::closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid)
    {
        if (kdTree != nullptr) {
            Vec3f coord;
            Vec3f pointModel = invert_affine(getModelMatrix()).mul_3D_point(point);
            float max = 0.1;
            if (kdTree->closest_point(pointModel, max, coord, vid)){
                return true;
            }
            return false;
        } else {
            //iterate over every face
            float distance = FLT_MAX;
            HMesh::VertexID closestVertex;
            Vec3f pointModel = Vec3f(invert_affine(getModelMatrix()) * Vec4f(point));
            
            if (no_vertices()==0) {
                return false;
            }

            for (VertexIDIterator vID = vertices_begin(); vID != vertices_end(); vID++)
            {
                Vec3f vertexPos = posf(*vID);
                float cur_distance = (pointModel - vertexPos).length();
                if (cur_distance < distance) {
                    distance = cur_distance;
                    vid = *vID;
                }
            }
            return true;
        }
    }
    
    void PAMManifold::buildKDTree()
    {
        kdTree = new KDTree<Vec3f, VertexID>();
        for(VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid) {
            kdTree->insert(posf(*vid), *vid);
        }
        kdTree->build();
    }
    
    void PAMManifold::draw() const
    {
        if (!enabled)
            return;
        
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        Mat3x3 normalMat = transpose(getNormalMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing PAM");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, normalMat.get());
        GL_CHECK_ERROR;

        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        normalDataBuffer->bind();
        normalDataBuffer->prepareToDraw(attrib[ATTRIB_NORMAL], 3, 0, GL_FLOAT, GL_FALSE);
        
        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);

        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.0f, 2.0f);

        indexDataBuffer->bind();
        indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);
        
        wireframeColorBuffer->bind();
        wireframeColorBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);

        glLineWidth(2.0f);
        glDisable(GL_POLYGON_OFFSET_FILL);
        wireframeIndexBuffer->bind();
        wireframeIndexBuffer->drawPreparedArraysIndicies(GL_LINES, GL_UNSIGNED_INT, numWireframeIndicies);

        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }
    
    void PAMManifold::drawToDepthBuffer() 
    {
        if (depthShaderProgram == nullptr) {
            std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"DepthShader" ofType:@"vsh"].UTF8String);
            std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"DepthShader" ofType:@"fsh"].UTF8String);
            
            depthShaderProgram = new RAES2ShaderProgram();
            depthShaderProgram->loadProgram(vShader_Cplus, fShader_Cplus);
            
            attribDepth[ATTRIB_POSITION] = depthShaderProgram->getAttributeLocation("aPosition");
            uniformsDepth[UNIFORM_MODELVIEWPROJECTION_MATRIX] = depthShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
        }
        
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        glPushGroupMarkerEXT(0, "Drawing PAM to Depth Buffer");
        
        glUseProgram(depthShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniformsDepth[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attribDepth[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        indexDataBuffer->bind();
        indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }

#pragma mark - MODELING FUNCTIONS
    
    bool PAMManifold::createBody(std::vector<CGLA::Vec3f>& polyline1,
                                 std::vector<CGLA::Vec3f>& polyline2,
                                 float zCoord,
                                 std::vector<std::vector<CGLA::Vec3f>>& debugAllRibs,
                                 bool debug)
    {
        assert(polyline1.size() == polyline2.size());
        if (polyline1.size() < 5) {
            RA_LOG_ERROR("Garbage point data");
            return false;
        }
        
        //CONVERT TO CAMERA COORDINATES. GET SKELETON.
        vector<Vec2f> polyline1_2d, polyline2_2d, skeleton;
        for (int i = 0; i < polyline1.size(); i++)
        {
            Vec3f vWorld1 = viewMatrix.mul_3D_point(polyline1[i]);
            Vec3f vWorld2 = viewMatrix.mul_3D_point(polyline2[i]);
            polyline1_2d.push_back(Vec2f(vWorld1)); //drop z-coord
            polyline2_2d.push_back(Vec2f(vWorld2));
            skeleton.push_back(0.5 * (polyline1_2d[i] + polyline2_2d[i]));
        }
        
        //STROKE FILTERING.
        float c_step = length(viewMatrix.mul_3D_vector(Vec3f(3*kCENTROID_STEP,0,0)));
        vector<Vec2f> r_polyline1, r_polyline2, r_skeleton;
        reduceLineToEqualSegments2D(r_polyline1, polyline1_2d, c_step);
        reduceLineToEqualSegments2D(r_polyline2, polyline2_2d, c_step);
        reduceLineToEqualSegments2D(r_skeleton, skeleton, c_step);
        
        if (r_skeleton.size() < 5) {
            RA_LOG_ERROR("Skeleton is too small");
            return false;
        }
        
        //SMOOTHING
        laplacianSmoothing(r_polyline1, r_polyline1, 2, 0.5);
        laplacianSmoothing(r_polyline2, r_polyline2, 2, 0.5);
        laplacianSmoothing(r_skeleton, r_skeleton, 50, 0.2);
        
        vector<Vec2f> s_skeleton, s_skeleton_norm, s_skeleton_tan;
        if (!getSmoothCurve(r_skeleton, &s_skeleton, &s_skeleton_tan, &s_skeleton_norm, c_step))
        {
            RA_LOG_ERROR("Failed to fit B-spline");
            return false;
        }
        else
        {
            if (s_skeleton.size() < 3) {
                RA_LOG_ERROR("Smoothed skeleton is too small");
                return false;
            }
        }

        //RIB WIDTHS
        vector<float> ribWidths;
        if (!getRibWidths(r_polyline1, r_polyline2, s_skeleton, s_skeleton_norm, ribWidths))
        {
            RA_LOG_ERROR("Failed to get rib widths");
            return false;
        }
        assert(ribWidths.size() == s_skeleton.size());
        if (ribWidths.size() != s_skeleton.size()) {
            RA_LOG_ERROR("Rib widths array has wrong size %lu expected %lu", ribWidths.size(), s_skeleton.size());
            return false;
        }

        //Parse new skeleton and create ribs
        //Ingore first and last centroids since they are poles
        int numSpines = 50;
        if (debug) {
            numSpines = 1;
        }

        vector<vector<Vec3f>> allRibs(s_skeleton.size());

        if (debug) //only for debuggin purposes
        {
            vector<Vec3f> s_polyline1_model, s_polyline2_model;
            for (int i = 0; i < r_polyline1.size(); i++) {
                Vec3f sModel1 = invert_affine(viewMatrix).mul_3D_point(Vec3f(r_polyline1[i], zCoord));
                s_polyline1_model.push_back(sModel1);
            }
            debugAllRibs.push_back(s_polyline1_model);
            
            for (int i = 0; i < r_polyline2.size(); i++) {
                Vec3f sModel1 = invert_affine(viewMatrix).mul_3D_point(Vec3f(r_polyline2[i], zCoord));
                s_polyline2_model.push_back(sModel1);
            }
            debugAllRibs.push_back(s_polyline2_model);
        }

        vector<Vec3f> s_skelton_model;
        for (int i = 0; i < s_skeleton.size(); i++)
        {
            Vec3f sModel = invert_affine(viewMatrix).mul_3D_point(Vec3f(s_skeleton[i], zCoord));
            s_skelton_model.push_back(sModel);
            
            float ribWidth = ribWidths[i];//0.15; //TODO
            Vec3f nModel = invert_affine(viewMatrix).mul_3D_vector(Vec3f(ribWidth * s_skeleton_norm[i], 0));
            Vec3f tModel = invert_affine(viewMatrix).mul_3D_vector(Vec3f(s_skeleton_tan[i], 0));
            
            if (i == 0)
            {
                vector<Vec3f> firstPole;
                firstPole.push_back(sModel);
                allRibs[0] = firstPole;
            }
            else if (i == s_skeleton.size() - 1)
            {
                vector<Vec3f> secondPole;
                secondPole.push_back(sModel);
                allRibs[i] = secondPole;
            }
            else
            {
                vector<Vec3f> ribs(numSpines);
                float rot_step = 360.0f/numSpines;
                for (int j = 0; j < numSpines; j++)
                {
                    float angle = j * rot_step;
                    Quatf quat;
                    quat.make_rot(angle*DEGREES_TO_RADIANS, normalize(tModel));
                    Vec3f newNorm = quat.get_Mat4x4f().mul_3D_vector(nModel);
                    Vec3f newRibPoint = sModel + newNorm;
                    ribs[j] = newRibPoint;

                    if (debug)
                    {
                        vector<Vec3f> norms(2);
                        vector<Vec3f> tan(2);
                        norms.push_back(sModel);
                        norms.push_back(newRibPoint);
                        tan.push_back(sModel);
                        tan.push_back(sModel + ribWidth*normalize(tModel));
                        debugAllRibs.push_back(norms);
//                    debugAllRibs.push_back(tan);
                    }
                }
                allRibs[i] = ribs;
            }
        }
        
        if (debug) {
            debugAllRibs.push_back(s_skelton_model);
        }
        
        populateManifold(allRibs);
        return true;
    }

    
    void PAMManifold::populateManifold(std::vector<std::vector<CGLA::Vec3f>>& allRibs)
    {
        vector<Vec3f> vertices;
        vector<int> faces;
        vector<int> indices;
        
        //Add all verticies
        for (int i = 0; i < allRibs.size(); i++) {
            vector<Vec3f> rib = allRibs[i];
            for (int j = 0; j < rib.size(); j++) {
                Vec3f v = rib[j];
                vertices.push_back(v);
            }
        }
        
        for (int i = 0; i < allRibs.size() - 1; i++) {
            
            if (i == 0) { //pole 1
                vector<Vec3f> pole = allRibs[i];
                vector<Vec3f> rib = allRibs[i+1];
                int poleIndex = 0;
                for (int j = 0; j < rib.size(); j++) {
                    indices.push_back(poleIndex);
                    if (j == rib.size() - 1) {
                        int index1 = indexForCentroid(1,j,allRibs.size(),rib.size());
                        int index2 = indexForCentroid(1,0,allRibs.size(),rib.size());
                        indices.push_back(index2);
                        indices.push_back(index1);
                    } else {
                        int index1 = indexForCentroid(1,j,allRibs.size(),rib.size());
                        int index2 = indexForCentroid(1,j+1,allRibs.size(),rib.size());
                        indices.push_back(index2);
                        indices.push_back(index1);
                    }
                    faces.push_back(3);
                }
            } else if (i == allRibs.size() - 2) { //pole 2
                vector<Vec3f> pole = allRibs[i+1];
                vector<Vec3f> rib = allRibs[i];
                int poleIndex = indexForCentroid(i+1,0,allRibs.size(),rib.size());
                for (int j = 0; j < rib.size(); j++) {
                    indices.push_back(poleIndex);
                    if (j == rib.size() - 1) {
                        int index1 = indexForCentroid(i,j,allRibs.size(),rib.size());
                        int index2 = indexForCentroid(i,0,allRibs.size(),rib.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                    } else {
                        int index1 = indexForCentroid(i,j,allRibs.size(),rib.size());
                        int index2 = indexForCentroid(i,j+1,allRibs.size(),rib.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                    }
                    faces.push_back(3);
                }
            } else {
                vector<Vec3f> rib1 = allRibs[i];
                vector<Vec3f> rib2 = allRibs[i+1];
                
                for (int j = 0; j < rib1.size(); j++) {
                    if (j == rib1.size() - 1) {
                        int index1 = indexForCentroid(i,j,allRibs.size(),rib1.size());
                        int index2 = indexForCentroid(i,0,allRibs.size(),rib1.size());
                        int index3 = indexForCentroid(i+1,0,allRibs.size(),rib1.size());
                        int index4 = indexForCentroid(i+1,j,allRibs.size(),rib1.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                        indices.push_back(index3);
                        indices.push_back(index4);
                    } else {
                        int index1 = indexForCentroid(i,j,allRibs.size(),rib1.size());
                        int index2 = indexForCentroid(i,j+1,allRibs.size(),rib1.size());
                        int index3 = indexForCentroid(i+1,j+1,allRibs.size(),rib1.size());
                        int index4 = indexForCentroid(i+1,j,allRibs.size(),rib1.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                        indices.push_back(index3);
                        indices.push_back(index4);
                    }
                    faces.push_back(4);
                }
            }
        }
        
        clear();
        build(vertices.size(),
                reinterpret_cast<float*>(&vertices[0]),
                faces.size(),
                &faces[0],
                &indices[0]);
    }
    
    int PAMManifold::indexForCentroid(int centeroid, int rib, int totalCentroid, int totalRib)
    {
        if (centeroid == 0) {
            return 0;
        } else if (centeroid == totalCentroid - 1) {
            return (totalCentroid - 2)*totalRib + 2 - 1;
        } else {
            return 1 + (centeroid - 1)*totalRib + rib;
        }
    }

}