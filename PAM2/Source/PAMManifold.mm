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
#include "RAPolylineUtilities.h"
#include "Quatf.h"
#include "Quatd.h"
#include "PAMUtilities.h"
#include "polarize.h"
#include "Mat3x3d.h"
#include "eigensolution.h"
#include "ArithMatFloat.h"
#include "PAMSettingsManager.h"
#include <queue>
#include "smooth.h"

#define kCENTROID_STEP 0.025f

namespace PAMMesh
{
    using namespace HMesh;
    using namespace RAEngine;
    using namespace CGLA;
    using namespace std;
    using namespace Geometry;
    
#pragma mark - CONSTRUCTOR/DESTRUCTOR
    PAMManifold::PAMManifold() : HMesh::Manifold()
    {
        kdTree = nullptr;
        modState = Modification::NONE;
    }
    
    void PAMManifold::copyDataFromManifold(HMesh::Manifold& mani)
    {
        this->kernel = mani.kernel;
        this->positions = mani.positions;
        modState = Modification::NONE;
        
        bufferVertexDataToGPU();
        traceEdgeInfo();
        buildKDTree();
    }
    
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
        bufferVertexDataToGPU();
        buildKDTree();        
        traceEdgeInfo();
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
    
    void PAMManifold::traceEdgeInfo()
    {
        edgeInfo = trace_spine_edges(*this);
    }
    
    void PAMManifold::getVertexData(CGLA::Vec3f*& vertexPositions,
                                    CGLA::Vec3f*& vertexNormals,
                                    CGLA::Vec4uc*& vertexColors,
                                    std::vector<unsigned int>*& indicies,
                                    CGLA::Vec4uc*& wireframeColor,
                                    std::vector<unsigned int>*& wireframeIndicies)
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
        vertexIDtoIndex = std::map<HMesh::VertexID, int>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
            vertexPositions[i] = posf(*vid);
            vertexNormals[i] = HMesh::normalf(*this, *vid);
            vertexColors[i] = color;
            wireframeColor[i] = wcolor;
            vertexIDtoIndex[*vid] = i;
        }
        
        for (FaceIDIterator fid = faces_begin(); fid != faces_end(); ++fid) {
            int vertexNum = 0;
            unsigned int facet[4];
            
            //iterate over every vertex of the face
            for (Walker w = walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
//                unsigned int index = vID.index;
                unsigned int index = vertexIDtoIndex[vID];
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
        
//        delete vertexIDtoIndex;
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
    
    void PAMManifold::buildKDTree()
    {
        if (kdTree != nullptr) {
            delete kdTree;
        }
        kdTree = new KDTree<Vec3f, VertexID>();
        for(VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid) {
            kdTree->insert(posf(*vid), *vid);
        }
        kdTree->build();
    }
    
    void PAMManifold::updateMesh()
    {
        if (modState == Modification::SCULPTING_SCALING)
        {
            for (int i = 0; i < _edges_to_scale.size(); i ++) {
                scaled_pos_for_rib(*this,
                                   _edges_to_scale[i],
                                   _centroids[i],
                                   edgeInfo,
                                   1 + (_scaleFactor - 1)*_scale_weight_vector[i],
                                   _current_scale_position);
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _sculpt_verticies_to_scale) {
                Vec3f pos = _current_scale_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            
        }
        else if (modState == Modification::SCULPTING_ANISOTROPIC_SCALING)
        {
            for (int i = 0; i < _edges_to_scale.size(); i ++)
            {
                HalfEdgeID ribID = _edges_to_scale[i];
                float scale =   1 + (_scaleFactor - 1)*_scale_weight_vector[i];
                for (Walker w = walker(ribID); !w.full_circle(); w = w.next().opp().next()) {
                    VertexID vID = w.vertex();
                    Vec3f proj = _anisotropic_projections[vID];
                    Vec3f newPos = posf(vID) + (scale - 1) * proj;
                    _current_scale_position[w.vertex()] = newPos;
                }
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _sculpt_verticies_to_scale) {
                Vec3f pos = _current_scale_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (modState == Modification::SCULPTING_BUMP_CREATION)
        {
            for (VertexID vid: _bump_verticies)
            {
                float weight = _bump_verticies_weigths[vid];
                float depth;
                if (_bumpBrushDepth < 1) {
                    depth = 1 - _bumpBrushDepth;
                } else {
                    depth = -1*(_bumpBrushDepth - 1);
                }
                _bump_current_displacement[vid] = Vec3f(pos(vid) + depth*_bumpDirection*weight);
            }
            
            for (VertexID v: _bump_verticies)
            {
                Vec3f p0 = _bump_current_displacement[v];
                
                vector<Vec3f> one_ring;
                
                // run through outgoing edges, and store them normalized
                circulate_vertex_ccw(*this, v, (std::function<void(VertexID)>)[&](VertexID vn) {
                    Vec3f edge;
                    if(_bump_current_displacement_vid_is_set[vn]) {
                        edge = _bump_current_displacement[vn] - p0;
                    } else {
                        edge = posf(vn) - p0;
                    }
                    
                    double l = length(edge);
                    if(l > 0.0)
                        one_ring.push_back(edge/l);
                });
                int N = one_ring.size();
                
                size_t N_count = N;
                size_t N_start = 0;
                if(boundary(*this, v))
                    N_start = 1;
                
                // sum up the normals of each face surrounding the vertex
                Vec3f n(0);
                for(size_t i = N_start; i < N_count; ++i){
                    Vec3f e0 = one_ring[i];
                    Vec3f e1 = one_ring[(i+1) % N];
                    
                    Vec3f n_part = normalize(cross(e0, e1));
                    n += n_part * acos(max(-1.0, fmin(1.0, dot(e0, e1))));
                }
                
                // normalize and return the normal
                float sqr_l = sqr_length(n);
                if(sqr_l > 0.0f)
                    n = n / sqrt(sqr_l);
                
                _bump_current_norms[v] = n;
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _bump_verticies) {
                Vec3f pos = _bump_current_displacement[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            
            normalDataBuffer->bind();
            unsigned char* tempnorm = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _bump_verticies) {
                Vec3f norm = _bump_current_norms[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(tempnorm + index*sizeof(Vec3f), norm.get(), sizeof(Vec3f));
            }
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (modState == Modification::BRANCH_ROTATION)
        {
            Vec3f zAxis = invert_affine(viewMatrix).mul_3D_vector(Vec3f(0, 0, -1));
            Mat4x4f toOrigin = translation_Mat4x4f(-1*_centerOfRotation);
            Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*_rotAngle);
            Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
            Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
            
            for (VertexID vID: _transformed_verticies) {
                _current_rot_position[vID] = tMatrix.mul_3D_point(posf(vID));
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _transformed_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            //deformable area
            map<int,Mat4x4f> rotMatricies;
            for (auto lid: _loopsToDeform) {
                float weight = _ringToDeformValue[lid];
                float angle = weight * -1*_rotAngle;
                Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, angle);
                Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
                rotMatricies[lid]=tMatrix;
            }
            
            for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
                VertexID vid = it->first;
                int lid = it->second;
                Mat4x4f tMatrix = rotMatricies[lid];
                Vec3f newPos = tMatrix.mul_3D_point(posf(vid));
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), newPos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (modState == Modification::BRANCH_SCALING)
        {
            Mat4x4f toOrigin = translation_Mat4x4f(-1 * _centerOfRotation);
            Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
            Mat4x4f scaleMatrix = scaling_Mat4x4f(Vec3f(_scaleFactor));
            Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
            
            for (VertexID vid: _transformed_verticies) {
                _current_rot_position[vid] = tMatrix.mul_3D_point(posf(vid));
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _transformed_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            //deformable area
            map<int,Mat4x4f> scaleMatricies;
            for (auto lid: _loopsToDeform) {
                float weight = _ringToDeformValue[lid];
                float scale = 1 + (_scaleFactor - 1)*weight;
                Mat4x4f sMatrix = scaling_Mat4x4f(Vec3f(scale));
                scaleMatricies[lid]=sMatrix;
            }
            
            for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
                VertexID vid = it->first;
                int lid = it->second;
                
                Mat4x4f scaleMatrix = scaleMatricies[lid];
                Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
                Vec3f newPos = tMatrix.mul_3D_point(posf(vid));
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), newPos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            
        }
        else if (modState == Modification::BRANCH_TRANSLATION)
        {
            Mat4x4f translationMatrix = translation_Mat4x4f(_translationCurrent);
            
            for (VertexID vid: _transformed_verticies) {
                _current_rot_position[vid] = translationMatrix.mul_3D_point(posf(vid));;
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _transformed_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            //deformable area
            map<int,Mat4x4f> scaleMatricies;
            for (auto lid: _loopsToDeform) {
                scaleMatricies[lid]= translation_Mat4x4f(_ringToDeformValue[lid] * _translationCurrent);;
            }
            
            for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
                VertexID vid = it->first;
                int lid = it->second;
                Vec3f newPos = scaleMatricies[lid].mul_3D_point(posf(vid));
                
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), newPos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (modState == Modification::BRANCH_DETACHED_ROTATE) {
            CGLA::Quatd q;
            q.make_rot(-_rotAngle, _zRotateVec);
            
            for (VertexID vid: _detached_verticies) {
                Vec p = pos(vid);
                p -= _zRotatePos;
                p = q.apply(p);
                p += _zRotatePos;
                _current_rot_position[vid] = Vec3f(p);
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _detached_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        } else if (modState == Modification::BRANCH_CLONE_ROTATION) {
            CGLA::Quatd q;
            q.make_rot(-_rotAngle, _zRotateVec);
            
            for (VertexID vid: _cloned_verticies) {
                Vec p = pos(vid);
                p -= _zRotatePos;
                p = q.apply(p);
                p += _zRotatePos;
                _current_rot_position[vid] = Vec3f(p);;
            }

            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _cloned_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        } else if (modState == Modification::BRANCH_CLONE_SCALING) {
            
            Mat4x4f toOrigin = translation_Mat4x4f(-1 * _centerOfRotation);
            Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
            Mat4x4f scaleMatrix = scaling_Mat4x4f(Vec3f(_scaleFactor));
            Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
            
            for (VertexID vid: _cloned_verticies) {
                _current_rot_position[vid] = tMatrix.mul_3D_point(posf(vid));
            }
            
            positionDataBuffer->bind();
            
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _cloned_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        else if (modState == Modification::BRANCH_POSE_ROTATE)
        {
            Vec3f zAxis = invert_affine(viewMatrix).mul_3D_vector(Vec3f(0, 0, -1));
            Mat4x4f toOrigin = translation_Mat4x4f(-1*_centerOfRotation);
            Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*_rotAngle);
            Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
            Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
            
            for (VertexID vID: _transformed_verticies) {
                _current_rot_position[vID] = tMatrix.mul_3D_point(posf(vID));;
            }
            
            positionDataBuffer->bind();
            unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
            for (VertexID vid: _transformed_verticies) {
                Vec3f pos = _current_rot_position[vid];
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), pos.get(), sizeof(Vec3f));
            }
            
            //deformable area
            map<int,Mat4x4f> rotMatricies;
            for (auto lid: _loopsToDeform) {
                float weight = _ringToDeformValue[lid];
                float angle = weight * _rotAngle;
                Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*angle);
                Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
                rotMatricies[lid]=tMatrix;
            }
            
            for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
                VertexID vid = it->first;
                int lid = it->second;
                Mat4x4f tMatrix = rotMatricies[lid];
                Vec3f newPos = tMatrix.mul_3D_point(posf(vid));
                
                int index = vertexIDtoIndex[vid];
                memcpy(temp + index*sizeof(Vec3f), newPos.get(), sizeof(Vec3f));
            }
            
            glUnmapBufferOES(GL_ARRAY_BUFFER);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }
    
    void PAMManifold::subdivide()
    {
        polar_subdivide(*this, 1);
        buildKDTree();
        bufferVertexDataToGPU();
        traceEdgeInfo();
    }
    
    void PAMManifold::globalSmoothing()
    {
        laplacian_smooth(*this, 1.0, 2);
        buildKDTree();
        bufferVertexDataToGPU();
    }
    
    void PAMManifold::showSkeleton(bool show)
    {
//        if (show) {
//            skeleton_retract(*this, 0.9);
//        } else {
//            skeleton_retract(*this, -0.9);
//        }
//        bufferVertexDataToGPU();
    }
    
    void PAMManifold::clearVertexData()
    {
        this->clear();
        bufferVertexDataToGPU();
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
        if (depthShaderProgram == nullptr)
        {
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

#pragma mark - UTILITIES
    
    bool PAMManifold::closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid)
    {
        if (kdTree != nullptr)
        {
            //try kd tree
            Vec3f coord;
            Vec3f pointModel = invert_affine(getModelMatrix()).mul_3D_point(point);
            float max = 0.1; //TODO
            if (kdTree->closest_point(pointModel, max, coord, vid)){
                return true;
            }
        }
        
        if (no_vertices() == 0) {
            return false;
        }
        
        //iterate over every face
        float distance = FLT_MAX;
        Vec3f pointModel = invert_affine(getModelMatrix()).mul_3D_point(point);
        
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
    
    bool PAMManifold::closestVertexID_2D(const CGLA::Vec3f& point, HMesh::VertexID& vid)
    {
        if (no_vertices() == 0) {
            return false;
        }
        
        float distance = FLT_MAX;
        
        //touchPoint to world coordinates
        Vec3f touchPoint = viewMatrix.mul_3D_point(point);
        Vec2f touchPoint2D = Vec2f(touchPoint[0], touchPoint[1]);
        
        for (VertexIDIterator vID = vertices_begin(); vID != vertices_end(); vID++)
        {
            Vec3f vertexPos = posf(*vID);
            Vec3f glkVertextPosModelView = getModelViewMatrix().mul_3D_point(vertexPos);
            Vec2f glkVertextPosModelView_2 = Vec2f(glkVertextPosModelView[0], glkVertextPosModelView[1]);
            float cur_distance = length(touchPoint2D - glkVertextPosModelView_2);
            
            if (cur_distance < distance) {
                distance = cur_distance;
                vid = *vID;
            }
        }
        return true;
    }
    
    std::set<HMesh::VertexID> PAMManifold::allVerticiesInDirectionHID(HMesh::HalfEdgeID hID)
    {
        Walker w = this->walker(hID);
        return allVerticiesInDirection(w);
    }
    
    std::set<HMesh::VertexID> PAMManifold::allVerticiesInDirection(HMesh::Walker deleteDir)
    {
        
        if (is_pole(*this, deleteDir.vertex())) {
            set<VertexID> floodVerticiesSet;
            floodVerticiesSet.insert(deleteDir.vertex());
            return floodVerticiesSet;
        }
        
        //Flood rotational area
        HalfEdgeAttributeVector<EdgeInfo> sEdgeInfo(allocated_halfedges());
        Walker bWalker = this->walker(deleteDir.next().halfedge()); //Walk along pivot boundary loop
        queue<HalfEdgeID> hq;
        
        for (;!bWalker.full_circle(); bWalker = bWalker.next().opp().next()) {
            HalfEdgeID hID = bWalker.next().halfedge();
            HalfEdgeID opp_hID = bWalker.next().opp().halfedge();
            sEdgeInfo[hID] = EdgeInfo(SPINE, 0);
            sEdgeInfo[opp_hID] = EdgeInfo(SPINE, 0);
            hq.push(hID);
        }
        
        set<VertexID> floodVerticiesSet;
        while(!hq.empty())
        {
            HalfEdgeID h = hq.front();
            Walker w = this->walker(h);
            hq.pop();
            bool is_spine = edgeInfo[h].edge_type == SPINE;
            for (;!w.full_circle(); w=w.circulate_vertex_ccw(),is_spine = !is_spine) {
                if(sEdgeInfo[w.halfedge()].edge_type == UNKNOWN)
                {
                    EdgeInfo ei = is_spine ? EdgeInfo(SPINE,0) : EdgeInfo(RIB,0);
                    
                    floodVerticiesSet.insert(w.vertex());
                    floodVerticiesSet.insert(w.opp().vertex());
                    
                    sEdgeInfo[w.halfedge()] = ei;
                    sEdgeInfo[w.opp().halfedge()] = ei;
                    hq.push(w.opp().halfedge());
                }
            }
        }
        return floodVerticiesSet;
    }


#pragma mark -  SMOOTHING
    //get all neighbouring points for the given verticies
    void PAMManifold::neighbours(std::set<HMesh::VertexID>& neighbours,
                                 std::vector<HMesh::VertexID>& verticies,
                                 float brush_size)
    {
        HalfEdgeAttributeVector<EdgeInfo> edge_info(allocated_halfedges());
        
        for (VertexID vID: verticies) {
            Vec originPos = pos(vID);
            queue<HalfEdgeID> hq;
            edge_info.clear();
            
            neighbours.insert(vID);
            circulate_vertex_ccw(*this, vID, [&](Walker w) {
                neighbours.insert(w.vertex());
                edge_info[w.halfedge()] = EdgeInfo(SPINE, 0);
                edge_info[w.opp().halfedge()] = EdgeInfo(SPINE, 0);
                hq.push(w.opp().halfedge());
            });
            
            while(!hq.empty())
            {
                HalfEdgeID h = hq.front();
                Walker w = walker(h);
                hq.pop();
                
                for (;!w.full_circle(); w = w.circulate_vertex_ccw()) {
                    if(edge_info[w.halfedge()].edge_type == UNKNOWN)
                    {
                        Vec p = pos(w.vertex());
                        float d = (p - originPos).length();
                        if (d <= brush_size) {
                            neighbours.insert(w.vertex());
                            //                        neighbours.insert(w.opp().vertex());
                            
                            edge_info[w.halfedge()] = EdgeInfo(SPINE,0);
                            edge_info[w.opp().halfedge()] = EdgeInfo(SPINE,0);
                            
                            hq.push(w.opp().halfedge());
                        }
                    }
                }
            }
        }
    }
    
    //get all neighbouring points for the given verticies
    void PAMManifold::neighbours(std::vector<HMesh::VertexID>& neighbours,
                                 HMesh::VertexID vID,
                                 std::vector<float>& weights,
                                 float brush_size)
    {
        HalfEdgeAttributeVector<EdgeInfo> edge_info(allocated_halfedges());
        
        Vec originPos = pos(vID);
        queue<HalfEdgeID> hq;
        edge_info.clear();
        
        neighbours.push_back(vID);
        weights.push_back(1.0);
        circulate_vertex_ccw(*this, vID, [&](Walker w) {
            neighbours.push_back(w.vertex());
            weights.push_back(1.0);
            edge_info[w.halfedge()] = EdgeInfo(SPINE, 0);
            edge_info[w.opp().halfedge()] = EdgeInfo(SPINE, 0);
            hq.push(w.opp().halfedge());
        });
        
        while(!hq.empty())
        {
            HalfEdgeID h = hq.front();
            Walker w = walker(h);
            hq.pop();
            
            for (;!w.full_circle(); w = w.circulate_vertex_ccw()) {
                if(edge_info[w.halfedge()].edge_type == UNKNOWN)
                {
                    Vec p = pos(w.vertex());
                    float d = (p - originPos).length();
                    if (d <= brush_size) {
                        float d = (p - originPos).length();
                        float x = d/brush_size;
                        float weight = pow(pow(x, 2) - 1, 2);
                        
                        weights.push_back(weight);
                        neighbours.push_back(w.vertex());
                        //                        neighbours.insert(w.opp().vertex());
                        
                        edge_info[w.halfedge()] = EdgeInfo(SPINE,0);
                        edge_info[w.opp().halfedge()] = EdgeInfo(SPINE,0);
                        
                        hq.push(w.opp().halfedge());
                    }
                }
            }
        }
        
    }
    
    //Smooth according to number of edges from the vertex
    void PAMManifold::smoothPole(HMesh::VertexID vID, int depth, int iter)
    {
        assert(is_pole(*this, vID));
        Walker w = walker(vID);
        int cur_depth = 0;
        vector<VertexID> allVert;
        allVert.push_back(vID);
        while (cur_depth <= depth) {
            vector<VertexID> vert = verticies_along_the_rib(*this, w.next().halfedge(), edgeInfo);
            allVert.insert(allVert.end(), vert.begin(), vert.end());
            w = w.next().opp().next();
            cur_depth++;
        }
        laplacian_spine_smooth_verticies(*this, allVert, edgeInfo, iter);
    }

    //Smooth verticies along the rib. If isSpine is YES, edge_info must be valid
    std::set<HMesh::VertexID> PAMManifold::smoothAlongRib(HMesh::HalfEdgeID rib,
                                                          int iter,
                                                          bool isSpine,
                                                          float brushSize)
    {
        vector<VertexID> vIDs;
        for (Walker w = walker(rib); !w.full_circle(); w = w.next().opp().next()) {
            vIDs.push_back(w.vertex());
        }
        set<VertexID> affectedVerticies = smoothVerticies(vIDs,iter,isSpine,brushSize);
        return affectedVerticies;
    }
    
    //Smooth at multiple verticies
    std::set<HMesh::VertexID> PAMManifold::smoothVerticies(std::vector<HMesh::VertexID> vIDs,
                                                           int iter,
                                                           bool isSpine,
                                                           float brushSize)
    {
        set<VertexID> allVerticiesSet;
        neighbours(allVerticiesSet,vIDs,brushSize);
        
        if (isSpine) {
            laplacian_spine_smooth_verticies(*this, allVerticiesSet, edgeInfo, iter);
        } else {
            laplacian_smooth_verticies(*this, allVerticiesSet, iter);
        }
        return allVerticiesSet;
    }
    
    void PAMManifold::smoothAtPoint(CGLA::Vec3f touchPoint, float radius, int iter)
    {
        VertexID closestPoint;
        closestVertexID_3D(touchPoint,closestPoint);
        
        vector<VertexID> affectedVerticies;
        vector<float> weights;
        neighbours(affectedVerticies,closestPoint,weights,radius);
        laplacian_smooth_verticies(*this, affectedVerticies, weights, iter);
        
        updateVertexPositionOnGPU_Vector(affectedVerticies);
        updateVertexNormOnGPU_Vector(affectedVerticies);
        //    [self changeVerticiesColor_Vector:prevAffectedVerticies toSelected:NO];
        //    [self changeVerticiesColor_Vector:affectedVerticies toSelected:YES];
        //        prevAffectedVerticies = affectedVerticies;
    }
    
#pragma mark - BODY CREATION
    
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
        buildKDTree();
        bufferVertexDataToGPU();
        traceEdgeInfo();
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

#pragma mark - BRANCH CREATION
    
    bool PAMManifold::createBranch(std::vector<CGLA::Vec3f> touchPoints,
                                   CGLA::Vec3f firstPoint,
                                   bool touchedModelStart,
                                   float touchSize,
                                   float angularWidth)
                                            
    {
        VertexID touchedVID;
        if (touchedModelStart) {
            closestVertexID_3D(firstPoint, touchedVID);
        } else {
            closestVertexID_2D(firstPoint, touchedVID);
        }
        
        VertexID poleID;
        if (touchedNearPole(touchedVID,poleID)) {
            return extendBranchAtPole(poleID, touchPoints);
        }
        
        if (touchPoints.size() < 6) {
            RA_LOG_WARN("Not enough points");
            return false;
        }
        
        //convert touch points to world space
        for_each(touchPoints.begin(), touchPoints.end(), [&](Vec3f &v){ v = viewMatrix.mul_3D_point(v);});

        //Get skeleton aka joint points
        vector<Vec3f> rawSkeleton;
        float c_step = length(viewMatrix.mul_3D_vector(Vec3f(3*kCENTROID_STEP,0,0)));
        reduceLineToEqualSegments3D(rawSkeleton, touchPoints, c_step);
        if (rawSkeleton.size() < 4) {
            RA_LOG_WARN("Not enough controids");
            return false;
        }

        int limbWidth = branchWidthForAngle(angularWidth*DEGREES_TO_RADIANS, touchedVID);
        if (limbWidth <= 1 ) {
            return false;
        }
        RA_LOG_INFO("Limb Width: %i", limbWidth);
        
//        [self saveState]; //TODO add save state
        //Create new pole
        VertexID newPoleID;
        float bWidth;
        Vec3f holeCenter, holeNorm;
        HalfEdgeID boundaryHalfEdge;
        bool result = createHoleAtVertex(touchedVID, limbWidth, newPoleID, bWidth,
                                         holeCenter, holeNorm, boundaryHalfEdge);
        assert(result);
        
        //closest to the first centroid between two fingers vertex in 2D space
        Mat4x4f mvMatrix = getModelViewMatrix();
        Vec3f holeCenterWorld = mvMatrix.mul_3D_point(holeCenter);
        //add depth to skeleton points. Interpolate if needed
        for_each(rawSkeleton.begin(), rawSkeleton.end(), [&](Vec3f &v){v = Vec3f(v[0], v[1], holeCenterWorld[2]);});
        
        //Smooth
        vector<Vec3f> skeleton;
        laplacianSmoothing(rawSkeleton, skeleton, 3, 0.5);
        
        //Skeleton should start from the branch point
        Vec3f translate = holeCenterWorld - skeleton[0];
        for_each(skeleton.begin(), skeleton.end(), [&](Vec3f &v){v += translate;});
        
        //Length
        float totalLength = 0;
        for (int i = 1; i < skeleton.size(); i++) {
            totalLength += length(skeleton[i-1] - skeleton[i]);
        }
        
        float deformLength = 0.1f * totalLength;
        int deformIndex = 0;
        totalLength = 0;
        for (int i = 1; i < skeleton.size(); i++) {
            totalLength += length(skeleton[i-1] - skeleton[i]);
            if (totalLength > deformLength) {
                deformIndex = i + 1;
                break;
            }
        }
        
        //Move branch by weighted norm
        Vec3f holeNormWorld = normalize(mvMatrix.mul_3D_vector(holeNorm));
        holeNormWorld = deformLength * holeNormWorld;
        for (int i = 0; i < skeleton.size(); i++) {
            if (i < deformIndex) {
                float x = (float)i/(float)deformIndex;
                float weight = sqrt(x);
                skeleton[i] = skeleton[i] + weight*holeNormWorld;
            } else {
                skeleton[i] = skeleton[i] + holeNormWorld;
            }
        }
        
        //Get norm vectors for skeleton joints
        vector<Vec3f> skeletonTangents;
        vector<Vec3f> skeletonNormals;
        normals3D(skeletonNormals, skeletonTangents,skeleton);
        
        //Parse new skeleton and create ribs
        //Ingore first and last centroids since they are poles
        int numSpines = limbWidth*2;
        vector<vector<Vec3f>> allRibs(skeleton.size());
        Mat4x4f invertModelViewMatrix = invert_affine(mvMatrix);
        
        for (int i = 0; i < skeleton.size(); i++) {
            Vec3f sModel = invertModelViewMatrix.mul_3D_point(skeleton[i]);
            float ribWidth = bWidth;
            Vec3f nModel = invertModelViewMatrix.mul_3D_vector(skeletonNormals[i]);
            Vec3f tModel = invertModelViewMatrix.mul_3D_vector(skeletonTangents[i]);
            tModel = ribWidth*normalize(tModel);
            nModel = ribWidth*normalize(nModel);
            
            if (i == skeleton.size() - 1) {
                vector<Vec3f> secondPole;
                secondPole.push_back(sModel);
                allRibs[i] = secondPole;
            } else {
                vector<Vec3f> ribs(numSpines);
                float rot_step = 360.0f/numSpines;
                Mat4x4f toOrigin = translation_Mat4x4f(-1*sModel);
                Mat4x4f fromOrigin = translation_Mat4x4f(sModel);
                
                for (int j = 0; j < numSpines; j++) {
                    float angle = j * rot_step;
                    
                    Mat4x4f rotMatrix = rotation_Mat4x4f(tModel, -1*angle*DEGREES_TO_RADIANS);
                    Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
                    
                    Vec3f startPosition = sModel + nModel;
                    startPosition =  tMatrix.mul_3D_point(startPosition);
                    ribs[j] = startPosition;
                }
                allRibs[i] = ribs;
            }
        }
        
        vector<Vec3f> newLimbVerticies;
        vector<int>  newLimbFaces;
        vector<int> newLimbIndices;
        populateNewLimb(allRibs,newLimbVerticies,newLimbFaces,newLimbIndices);
        
        assert(newLimbVerticies.size() != 0);
        assert(newLimbFaces.size() != 0);
        assert(newLimbIndices.size() != 0);
        
        FaceIDIterator lastFace = faces_end();
        build(newLimbVerticies.size(),
              reinterpret_cast<float*>(&newLimbVerticies[0]),
              newLimbFaces.size(),
              &newLimbFaces[0],
              &newLimbIndices[0]);
        
        lastFace++;
        Walker w = walker(*lastFace);
        
        vector<HalfEdgeID> newEdges;
        vector<VertexID> newVerticies;
        allVerticiesAndHalfEdges(newVerticies,newEdges,w.vertex());
        
        HalfEdgeID newBranchLowerRibEdge;
        assert(boundaryHalfEdgeForClonedMesh(newBranchLowerRibEdge,newEdges));
        
        Walker lowerBoundaryWalker = walker(newBranchLowerRibEdge);
        HalfEdgeID newBranchUpperRibEdge = lowerBoundaryWalker.opp().halfedge();
        stitchBranchToBody(boundaryHalfEdge,newBranchLowerRibEdge);
        
        //smooth at the bottom
        float smoothinCoefficient = PAMSettingsManager::getInstance().smoothingBrushSize;
        int iterations = PAMSettingsManager::getInstance().baseSmoothingIterations;
        traceEdgeInfo();

        float radius = rib_radius(*this, newBranchUpperRibEdge, edgeInfo);
        set<VertexID> affectedVerticies;
        if (!PAMSettingsManager::getInstance().spineSmoothing) {
            affectedVerticies = smoothAlongRib(newBranchUpperRibEdge,
                                               iterations,
                                               false,
                                               smoothinCoefficient*radius);
        } else {
            affectedVerticies = smoothAlongRib(newBranchUpperRibEdge,
                                               iterations,
                                               true,
                                               smoothinCoefficient*radius);
        }
        
        //smooth at the pole if needed
        if (PAMSettingsManager::getInstance().poleSmoothing) {
            for (VertexID vID: newVerticies) {
                if (is_pole(*this, vID)) {
                    Walker wBaseEnd = walker(vID);
                    HalfEdgeID pole_rib = wBaseEnd.next().halfedge();
                    float radius = rib_radius(*this, pole_rib, edgeInfo);
                    vector<HMesh::VertexID> verticiesToSmooth;
                    for (Walker w = walker(pole_rib); !w.full_circle(); w = w.next().opp().next()) {
                        verticiesToSmooth.push_back(w.vertex());
                    }
                    verticiesToSmooth.push_back(vID);
                    smoothVerticies(verticiesToSmooth,7,true,radius);
                    break;
                }
            }
        }
        
        buildKDTree();
        bufferVertexDataToGPU();
        traceEdgeInfo();
        
        return true;
    }
    
    bool PAMManifold::extendBranchAtPole(HMesh::VertexID poleID, std::vector<CGLA::Vec3f>& touchPoints)
    {
        assert(is_pole(*this, poleID));
        
        Vec3f polePos = posf(poleID);
        Walker fromPoleWalker = walker(poleID);
        HalfEdgeID upperRibID = fromPoleWalker.next().halfedge();
        HalfEdgeID lowerRibID = fromPoleWalker.next().opp().halfedge();
        Vec3f nextRibCenter = centroid_for_rib(*this, lowerRibID, edgeInfo);
        
        int poleValency = valency(*this, poleID);
        float ribWidth = rib_radius(*this, lowerRibID, edgeInfo);
        
        //    Vec ribRadiusVec = _manifold.pos(fromPoleWalker.vertex()) - nextRibCenter;
        //    normalize(ribRadiusVec);
        
        Vec3f poleNorm = polePos - nextRibCenter;
        poleNorm.normalize();
        
        if (touchPoints.size() < 6 ) {
            RA_LOG_WARN("Garbage point data");
            return false;
        }
        
        //convert touch points to world space
        for_each(touchPoints.begin(), touchPoints.end(), [&](Vec3f &v){ v = viewMatrix.mul_3D_point(v);});
        
        //Get skeleton aka joint points
        vector<Vec3f> rawSkeleton;
        float c_step = length(viewMatrix.mul_3D_vector(Vec3f(3*kCENTROID_STEP,0,0)));
        reduceLineToEqualSegments3D(rawSkeleton, touchPoints, c_step);
        if (rawSkeleton.size() < 4) {
            RA_LOG_WARN("Not enough controids");
            return false;
        }
        
//        [self saveState];
        
        //add depth
        Mat4x4f mvMatrix = getModelViewMatrix();
        Vec3f touchedV_world = mvMatrix.mul_3D_point(polePos);
        float zValueTouched = touchedV_world[2];
        for_each(rawSkeleton.begin(), rawSkeleton.end(), [&](Vec3f &v){v = Vec3f(v[0], v[1], zValueTouched);});
        
        //Smooth
        vector<Vec3f> skeleton;
        laplacianSmoothing(rawSkeleton, skeleton, 3, 0.5);
        
        //Skeleton should start at the centroid of the rib loop next to pole
        Vec3f translate = touchedV_world - skeleton[0];
        for_each(skeleton.begin(), skeleton.end(), [&](Vec3f &v){v += translate;});
        
        //Length
        float totalLength = 0;
        for (int i = 1; i < skeleton.size(); i++) {
            totalLength += length(skeleton[i-1] - skeleton[i]);
        }
        
        float deformLength = 0.1f * totalLength;
        int deformIndex = 0;
        totalLength = 0;
        for (int i = 1; i < skeleton.size(); i++) {
            totalLength += length(skeleton[i-1] - skeleton[i]);
            if (totalLength > deformLength) {
                deformIndex = i + 1;
                break;
            }
        }
        
        //Move branch by weighted norm
        Vec3f holeNormWorld = normalize(mvMatrix.mul_3D_vector(poleNorm));
        holeNormWorld = deformLength * holeNormWorld;
        for (int i = 0; i < skeleton.size(); i++) {
            if (i < deformIndex) {
                float x = (float)i/(float)deformIndex;
                float weight = sqrt(x);
                skeleton[i] = skeleton[i] + weight*holeNormWorld;
            } else {
                skeleton[i] = skeleton[i] + holeNormWorld;
            }
        }
        
        //Get norm vectors for skeleton joints
        vector<Vec3f> skeletonTangents;
        vector<Vec3f> skeletonNormals;
        normals3D(skeletonNormals, skeletonTangents,skeleton);
        
        //Parse new skeleton and create ribs
        //Ingore first and last centroids since they are poles
        int numSpines = poleValency;
        vector<vector<Vec3f>> allRibs(skeleton.size());
        Mat4x4f invertModelViewMatrix = invert_affine(mvMatrix);
        
        for (int i = 0; i < skeleton.size(); i++) {
            Vec3f sModel = invertModelViewMatrix.mul_3D_point(skeleton[i]);
            Vec3f nModel = invertModelViewMatrix.mul_3D_vector(skeletonNormals[i]);
            Vec3f tModel = invertModelViewMatrix.mul_3D_vector(skeletonTangents[i]);
            tModel = ribWidth*normalize(tModel);
            nModel = ribWidth*normalize(nModel);
            
            if (i == skeleton.size() - 1) {
                vector<Vec3f> secondPole;
                secondPole.push_back(sModel);
                allRibs[i] = secondPole;
            } else {
                vector<Vec3f> ribs(numSpines);
                float rot_step = 360.0f/numSpines;
                Mat4x4f toOrigin = translation_Mat4x4f(-1*sModel);
                Mat4x4f fromOrigin = translation_Mat4x4f(sModel);

                for (int j = 0; j < numSpines; j++) {
                    float angle = j * rot_step;
                    
                    Mat4x4f rotMatrix = rotation_Mat4x4f(tModel, -1*angle*DEGREES_TO_RADIANS);
                    Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
                    
                    Vec3f startPosition = sModel + nModel;
                    startPosition =  tMatrix.mul_3D_point(startPosition);
                    ribs[j] = startPosition;
                }
                allRibs[i] = ribs;
            }
        }

        //Delete pole
        remove_vertex(poleID);
        
        vector<Vec3f> newLimbVerticies;
        vector<int>  newLimbFaces;
        vector<int> newLimbIndices;
        populateNewLimb(allRibs,newLimbVerticies,newLimbFaces,newLimbIndices);
        
        assert(newLimbVerticies.size() != 0);
        assert(newLimbFaces.size() != 0);
        assert(newLimbIndices.size() != 0);
        
        FaceIDIterator lastFace = faces_end();
        build(newLimbVerticies.size(),
              reinterpret_cast<float*>(&newLimbVerticies[0]),
              newLimbFaces.size(),
              &newLimbFaces[0],
              &newLimbIndices[0]);
        
        lastFace++;
        Walker lastFaceWalker = walker(*lastFace);
        
        vector<HalfEdgeID> newEdges;
        vector<VertexID> newVerticies;
        allVerticiesAndHalfEdges(newVerticies,newEdges,lastFaceWalker.vertex());
        
        HalfEdgeID newBranchLowerRibEdge;
        assert(boundaryHalfEdgeForClonedMesh(newBranchLowerRibEdge,newEdges));
        
        stitchBranchToBody(upperRibID,newBranchLowerRibEdge);
        
        traceEdgeInfo();
        smoothAlongRib(lowerRibID,2,true,ribWidth);
        
        bufferVertexDataToGPU();
        buildKDTree();

        return true;
    }
    
    int PAMManifold::branchWidthForAngle(float angle, HMesh::VertexID vID)
    {
        
        Walker walker = this->walker(vID);
        if (edgeInfo[walker.halfedge()].is_spine()) {
            walker = walker.opp().next();
        }
        assert(edgeInfo[walker.halfedge()].is_rib());
        
        Vec3f centr = centroid_for_rib(*this, walker.halfedge(), edgeInfo);
        Vec3f v1 = posf(vID) - centr;
        
        float cur_angle = 0;
        int width = 0;
        while (cur_angle < angle/2) {
            Vec3f v2 = posf(walker.vertex()) - centr;
            float dotP = dot(normalize(v1), normalize(v2));
            cur_angle = acos(dotP);
            width += 1;
            walker = walker.next().opp().next();
        }
        
        return 2*width + 1;
    }
    
    bool PAMManifold::createHoleAtVertex(HMesh::VertexID vID,
                                         int width,
                                         HMesh::VertexID& newPoleID,
                                         float& bWidth,
                                         CGLA::Vec3f& holeCenter,
                                         CGLA::Vec3f& holeNorm,
                                         HMesh::HalfEdgeID& boundayHalfEdge)
    {
        bool result = createBranchAtVertex(vID,width,newPoleID,bWidth);
        if (result) {
            holeCenter  = posf(newPoleID);
            holeNorm  = normalf(*this, newPoleID);

            Walker w = this->walker(newPoleID);
            w = w.next();
            boundayHalfEdge = w.halfedge();
            
            RA_LOG_INFO("%i", valency(*this, w.vertex()));
            this->remove_vertex(newPoleID);
            
            return true;
        }
        return false;
    }
    
    bool PAMManifold::createBranchAtVertex(HMesh::VertexID vID,
                                           int numOfSegments,
                                           HMesh::VertexID& newPoleID,
                                           float& bWidth)
    {
        //Do not add branhes at poles
        if (is_pole(*this, vID)) {
            RA_LOG_WARN("Tried to create a branch at a pole");
            return false;
        }
        
        if (valency(*this, vID) > 4) {
            RA_LOG_WARN("Tried to create a branch at a rib junction");
            return false;
        }
        
        int leftWidth;
        int rightWidth;
        if (numOfSegments%2 != 0) {
            leftWidth = numOfSegments/2;
            rightWidth = numOfSegments/2 + 1;
        } else {
            leftWidth = numOfSegments/2;
            rightWidth = numOfSegments/2;
        }
        
        //Find rib halfedge that points to a given vertex
        Walker walker = this->walker(vID).opp();
        if (edgeInfo[walker.halfedge()].edge_type == SPINE) {
            walker = walker.next().opp();
        }
        assert(edgeInfo[walker.halfedge()].is_rib()); //its a rib
        assert(walker.vertex() == vID); //points to a given vertex
        
        //Check that rib ring has enough verticeis to accomodate branch width
        int num_of_rib_verticies = 0;
        for (Walker ribWalker = this->walker(walker.halfedge());
             !ribWalker.full_circle();
             ribWalker = ribWalker.next().opp().next(), num_of_rib_verticies++);
        
        if (num_of_rib_verticies < numOfSegments) {
            RA_LOG_WARN("Not enough points to create branh");
            return NO;
        }
        
        VertexAttributeVector<int> vs(no_vertices(), 0);
        vs[vID] = 1;
        
        vector<VertexID> ribs;
        
        //walk right
        int num_rib_found = 0;
        for (Walker ribWalker = walker.next().opp().next();
             num_rib_found < leftWidth;
             ribWalker = ribWalker.next().opp().next(), num_rib_found++)
        {
            ribs.push_back(ribWalker.vertex());
        }
        
        //walk left
        num_rib_found = 0;
        for (Walker ribWalker = walker.opp();
             num_rib_found < rightWidth;
             ribWalker = ribWalker.next().opp().next(), num_rib_found++)
        {
            ribs.push_back(ribWalker.vertex());
        }
        
        //Set all verticies to be branched out
        for (int i = 0; i < ribs.size(); i++) {
            VertexID cur_vID = ribs[i];
            vs[cur_vID] = 1;
        }
        
        newPoleID = polar_add_branch(*this, vs);
        refine_branch(*this, newPoleID, bWidth);
        
        return YES;
    }
    
    void PAMManifold::populateNewLimb(std::vector<std::vector<CGLA::Vec3f>>& allRibs,
                                      std::vector<CGLA::Vec3f>& vertices,
                                      std::vector<int>& faces,
                                      std::vector<int>& indices)
    {
    
        Vec3f poleVec;
        
        //Add all verticies
        for (int i = 0; i < allRibs.size(); i++) {
            vector<Vec3f> rib = allRibs[i];
            for (int j = 0; j < rib.size(); j++) {
                Vec3f v = rib[j];
                vertices.push_back(v);
            }
        }
        
        for (int i = 0; i < allRibs.size() - 1; i++) {
            if (i == allRibs.size() - 2) { //pole 2
                vector<Vec3f> pole = allRibs[i+1];
                vector<Vec3f> rib = allRibs[i];
                int poleIndex = limbIndexForCentroid(i+1,0,allRibs.size(),rib.size());
//                Vec3f pV = vertices[poleIndex];
//                poleVec = Vec3f(pV[0], pV[1], pV[2]);
                
                for (int j = 0; j < rib.size(); j++) {
                    indices.push_back(poleIndex);
                    if (j == rib.size() - 1) {
                        int index1 = limbIndexForCentroid(i,j,allRibs.size(),rib.size());
                        int index2 = limbIndexForCentroid(i,0,allRibs.size(),rib.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                    } else {
                        int index1 = limbIndexForCentroid(i,j,allRibs.size(),rib.size());
                        int index2 = limbIndexForCentroid(i,j+1,allRibs.size(),rib.size());
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
                        int index1 = limbIndexForCentroid(i,j,allRibs.size(),rib1.size());
                        int index2 = limbIndexForCentroid(i,0,allRibs.size(),rib1.size());
                        int index3 = limbIndexForCentroid(i+1,0,allRibs.size(),rib1.size());
                        int index4 = limbIndexForCentroid(i+1,j,allRibs.size(),rib1.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                        indices.push_back(index3);
                        indices.push_back(index4);
                    } else {
                        int index1 = limbIndexForCentroid(i,j,allRibs.size(),rib1.size());
                        int index2 = limbIndexForCentroid(i,j+1,allRibs.size(),rib1.size());
                        int index3 = limbIndexForCentroid(i+1, j+1,allRibs.size(),rib1.size());
                        int index4 = limbIndexForCentroid(i+1, j,allRibs.size(),rib1.size());
                        indices.push_back(index1);
                        indices.push_back(index2);
                        indices.push_back(index3);
                        indices.push_back(index4);
                    }
                    faces.push_back(4);
                
                }
            
            }
        }
    }
    
    int PAMManifold::limbIndexForCentroid(int centeroid,int rib,int totalCentroid, int totalRib)
    {
        if (centeroid == totalCentroid - 1) {
            return (totalCentroid - 1)*totalRib + 1 - 1;
        } else {
            return centeroid*totalRib + rib;
        }
    }


    void PAMManifold::allVerticiesAndHalfEdges(std::vector<HMesh::VertexID>& verticies,
                                               std::vector<HMesh::HalfEdgeID>& halfedges,
                                               HMesh::VertexID vID)
    {
        //Flood rotational area
        HalfEdgeAttributeVector<EdgeInfo> sEdgeInfo(this->allocated_halfedges());
        queue<HalfEdgeID> hq;
        
        set<HalfEdgeID> floodEdgesSet;
        circulate_vertex_ccw(*this, vID, [&](Walker w) {
            sEdgeInfo[w.halfedge()] = EdgeInfo(SPINE, 0);
            sEdgeInfo[w.opp().halfedge()] = EdgeInfo(SPINE, 0);
            floodEdgesSet.insert(w.halfedge());
            floodEdgesSet.insert(w.opp().halfedge());
            hq.push(w.opp().halfedge());
        });
        
        set<VertexID> floodVerticiesSet;
        floodVerticiesSet.insert(vID);
        while(!hq.empty())
        {
            HalfEdgeID h = hq.front();
            Walker w = walker(h);
            hq.pop();
            bool is_spine = edgeInfo[h].edge_type == SPINE;
            for (;!w.full_circle(); w=w.circulate_vertex_ccw(),is_spine = !is_spine) {
                if(sEdgeInfo[w.halfedge()].edge_type == UNKNOWN)
                {
                    EdgeInfo ei = is_spine ? EdgeInfo(SPINE,0) : EdgeInfo(RIB,0);
                    
                    floodVerticiesSet.insert(w.vertex());
                    floodVerticiesSet.insert(w.opp().vertex());
                    
                    floodEdgesSet.insert(w.halfedge());
                    floodEdgesSet.insert(w.opp().halfedge());
                    
                    sEdgeInfo[w.halfedge()] = ei;
                    sEdgeInfo[w.opp().halfedge()] = ei;
                    
                    hq.push(w.opp().halfedge());
                }
            }
        }
        
        vector<VertexID> floodVerticiesVector(floodVerticiesSet.begin(), floodVerticiesSet.end());
        verticies = floodVerticiesVector;
        
        vector<HalfEdgeID> floodEdgesVector(floodEdgesSet.begin(), floodEdgesSet.end());
        halfedges = floodEdgesVector;
    }
    
    bool PAMManifold::boundaryHalfEdgeForClonedMesh(HMesh::HalfEdgeID& boundaryHalfedge,
                                                    std::vector<HMesh::HalfEdgeID>& newHalfEdges)
    {
        for (HalfEdgeID hID: newHalfEdges) {
            Walker w = walker(hID);
            if (w.face() == InvalidFaceID) {
                boundaryHalfedge = w.halfedge();
                return true;
            }
        }
        return false;
    }
    
    void PAMManifold::stitchBranchToBody(HMesh::HalfEdgeID branchHID,HMesh::HalfEdgeID bodyHID)
    {
        //Align edges
        Walker align1 = walker(branchHID);
        Vec branchCenter = 0.5*(pos(align1.vertex()) + pos(align1.opp().vertex()));
        
        HalfEdgeID closestHID;
        float closestDist = FLT_MAX;
        for (Walker align2 = walker(bodyHID); !align2.next().full_circle(); align2 = align2.next()) {
            Vec bodyCenter = 0.5*(pos(align2.vertex()) + pos(align2.opp().vertex()));
            float cur_dist = (bodyCenter - branchCenter).length();
            
            if (cur_dist < closestDist) {
                closestDist = cur_dist;
                closestHID = align2.halfedge();
            }
        }
        
        //Stich boundary edges
        vector<HalfEdgeID> bEdges1;
        vector<HalfEdgeID> bEdges2;
        for (Walker stitch1 = walker(closestHID);!stitch1.full_circle(); stitch1 = stitch1.next())
        {
            bEdges1.push_back(stitch1.halfedge());
        }
        bEdges1.pop_back();
        
        for (Walker stitch2 = walker(branchHID); !stitch2.full_circle(); stitch2 = stitch2.prev()) {
            bEdges2.push_back(stitch2.halfedge());
        }
        bEdges2.pop_back();
        
        assert(bEdges1.size() == bEdges2.size());
        for (int i = 0; i < bEdges1.size() ; i++) {
            bool didStich = stitch_boundary_edges(bEdges1[i], bEdges2[i]);
            assert(didStich);
        }
    }
    
    bool PAMManifold::touchedNearPole(HMesh::VertexID touchID, HMesh::VertexID& poleID)
    {
        if (is_pole(*this, touchID)) {
            poleID = touchID;
            return YES;
        }
        
        bool foundPole = false;
        circulate_vertex_ccw(*this, touchID, [&](Walker w) {
            if (is_pole(*this, w.vertex())) {
                foundPole = true;
                poleID = w.vertex();
            }
        });
        
        if (foundPole) {
            return true;
        }
        
        return false;
    }
 
#pragma mark - BUMP CREATION
    void PAMManifold::startBumpCreation(CGLA::Vec3f touchPoint,
                                        float brushSize,
                                        float brushDepth)
    {
        VertexID touchedVID;
        closestVertexID_3D(touchPoint, touchedVID);
        Vec touchedPos = pos(touchedVID);
        Vec norm = HMesh::normal(*this, touchedVID);
        Vec displace = 0.05*norm;
        
        _bumpDirection = displace;
        _bumpBrushDepth = brushDepth;
        _bump_verticies_weigths = VertexAttributeVector<float>(no_vertices());
        _bump_current_displacement = VertexAttributeVector<Vec3f>(no_vertices());
        _bump_current_displacement_vid_is_set = VertexAttributeVector<int>(no_vertices(), 0);
        _bump_current_norms = VertexAttributeVector<Vec3f>(no_vertices());
        _bump_verticies.clear();
        
        vector<VertexID> oneVertex;
        oneVertex.push_back(touchedVID);
        neighbours(_bump_verticies, oneVertex,brushSize);
        
        for (VertexID vid: _bump_verticies)
        {
            double l = (touchedPos - pos(vid)).length();
            float x = l/brushSize;
            if (x <= 1) {
                float weight = pow(pow(x,2)-1, 2);
                _bump_verticies_weigths[vid] = weight;
            }
            _bump_current_displacement_vid_is_set[vid] = 1;
        }
        //    [self changeVerticiesColor_Set:_bump_verticies toSelected:YES];
        modState = Modification::SCULPTING_BUMP_CREATION;
    }
    
    void PAMManifold::continueBumpCreation(float brushDepth)
    {
        _bumpBrushDepth = brushDepth;
    }
    
    void PAMManifold::endBumpCreation()
    {
//        [self saveState];
        
        for (VertexID vid: _bump_verticies)
        {
            pos(vid) = Vec(_bump_current_displacement[vid]);
        }
        
        updateVertexPositionOnGPU_Set(_bump_verticies);
        updateVertexNormOnGPU_Set(_bump_verticies);

        modState = Modification::NONE;
    }

    
#pragma mark - RIB SCALING
    void PAMManifold::startScalingSingleRib(CGLA::Vec3f touchPoint,
                                            bool secondPointOnTheModel,
                                            float scale,
                                            float touchSize,
                                            bool anisotropic)
    {
        VertexID vID;
        if (!closestVertexID_3D(touchPoint, vID)) {
            return;
        }

        if (is_pole(*this, vID)) {
            return;
        }
        
        Walker ribWalker = walker(vID).opp();
        if (edgeInfo[ribWalker.halfedge()].is_spine()) {
            ribWalker = ribWalker.next().opp();
        }
        assert(edgeInfo[ribWalker.halfedge()].is_rib());
        assert(ribWalker.vertex() == vID);
        
        _edges_to_scale.clear();
        _sculpt_verticies_to_scale.clear();
        
        Walker upWalker = walker(ribWalker.next().halfedge());
        Walker downWalker = walker(ribWalker.opp().prev().opp().halfedge());
        assert(upWalker.opp().vertex() == downWalker.opp().vertex());
        
        Vec3f origin = posf(vID);
        float brushSize = touchSize;
        vector<float> allDistances;
        vector<Vec3f> silhouette_Verticies;
        vector<VertexID> vector_vid;
        vector<vector<VertexID>> verticies_along_ribs;
        float distance = (origin - posf(upWalker.vertex())).length();
        while (distance <= brushSize)
        {
            if (is_pole(*this, upWalker.vertex())) {
                break;
            }
            HalfEdgeID ribID = upWalker.next().halfedge();
            _edges_to_scale.push_back(ribID);
            vector_vid = verticies_along_the_rib(*this, ribID, edgeInfo);
            verticies_along_ribs.push_back(vector_vid);
            _sculpt_verticies_to_scale.insert(_sculpt_verticies_to_scale.end(), vector_vid.begin(), vector_vid.end());
            allDistances.push_back(distance);
            Vec3f silhouette = posf(upWalker.vertex());
            silhouette_Verticies.push_back(silhouette);
            
            upWalker = upWalker.next().opp().next();
            distance = (origin - posf(upWalker.vertex())).length();
        }
        
        HalfEdgeID ribID = ribWalker.halfedge();
        distance = 0;
        allDistances.push_back(distance);
        _edges_to_scale.push_back(ribID);
        vector_vid = verticies_along_the_rib(*this, ribID, edgeInfo);
        verticies_along_ribs.push_back(vector_vid);
        _sculpt_verticies_to_scale.insert(_sculpt_verticies_to_scale.end(), vector_vid.begin(), vector_vid.end());
        Vec3f silhouette = origin;
        silhouette_Verticies.push_back(silhouette);
        
        distance = (origin - posf(downWalker.vertex())).length();
        while (distance <= brushSize) {
            if (is_pole(*this, downWalker.vertex())) {
                break;
            }
            HalfEdgeID ribID = downWalker.next().halfedge();
            _edges_to_scale.push_back(ribID);
            vector_vid = verticies_along_the_rib(*this, ribID, edgeInfo);
            verticies_along_ribs.push_back(vector_vid);
            _sculpt_verticies_to_scale.insert(_sculpt_verticies_to_scale.end(), vector_vid.begin(), vector_vid.end());
            allDistances.push_back(distance);
            Vec3f silhouette = posf(downWalker.vertex());
            silhouette_Verticies.push_back(silhouette);
            
            downWalker = downWalker.next().opp().next();
            distance = (origin - posf(downWalker.vertex())).length();
        }
        
        _current_scale_position = VertexAttributeVector<Vec3f>(no_vertices());
        
        _scaleFactor = scale;
        
        //Calculate centroids and gaussian weights
        _centroids = centroid_for_ribs(*this, _edges_to_scale, edgeInfo);
        assert(_centroids.size() == allDistances.size());
        
        _scale_weight_vector.clear();
        if (_centroids.size() == 1) {
            _scale_weight_vector.push_back(1.0f);
        } else {
            for(int i = 0; i < _centroids.size(); i++)
            {
                float distance = allDistances[i];
                float x = distance/brushSize;
                float weight;
                if (x <= 1) {
                    weight = pow(pow(x, 2) - 1, 2);
                } else {
                    weight = 0;
                }
                _scale_weight_vector.push_back(weight);
            }
        }
        
        if (anisotropic)
        {
            _anisotropic_projections = VertexAttributeVector<Vec3f>(no_vertices());
            
            for(int i = 0; i < _centroids.size(); i++)
            {
                Vec3d center = Vec3d(_centroids[i]);
                vector<VertexID> verticies = verticies_along_ribs[i];
                
                Mat3x3d cov(0);
                for (int i = 0; i < verticies.size(); i++) {
                    VertexID vID = verticies[i];
                    Vec3d p = pos(vID);
                    Vec3d d = p - center;
                    Mat3x3d m;
                    outer_product(d,d,m);
                    cov += m;
                }
                
                Mat3x3d Q, L;
                int sol = power_eigensolution(cov, Q, L);
                
                Vec3d n;
                assert(sol >= 2);
                n = normalize(cross(Q[0],Q[1]));
                
                Vec3f nGLK = Vec3f(n);
                Vec3f nGLKWorld = getModelViewMatrix().mul_3D_vector(nGLK);
                Vec3f zWorld = Vec3f(0, 0, 1);
                Vec3f to_silhouette_axis_world = cross(zWorld, nGLKWorld);
                
                Vec3f to_silhouette_axis_model = invert_affine(getModelViewMatrix()).mul_3D_vector(to_silhouette_axis_world);
                to_silhouette_axis_model = normalize(to_silhouette_axis_model);
                Vec to_silhouette_axis = Vec(to_silhouette_axis_model[0],
                                             to_silhouette_axis_model[1],
                                             to_silhouette_axis_model[2]);
                
                HalfEdgeID ribID = _edges_to_scale[i];
                for (Walker w = walker(ribID); !w.full_circle(); w = w.next().opp().next())
                {
                    Vec p = pos(w.vertex()) - center;
                    float c = dot(p, to_silhouette_axis)/dot(to_silhouette_axis, to_silhouette_axis);
                    Vec proj = c * to_silhouette_axis;
                    if (secondPointOnTheModel) {
                        Vec toOrg = Vec(origin) - Vec(center);
                        if (dot(toOrg, proj) > 0) {
                            proj = Vec(0,0,0);
                        }
                    }
                    
                    _anisotropic_projections[w.vertex()] = Vec3f(proj);
                }
            }
        }
        
        
        if (anisotropic) {
            modState = Modification::SCULPTING_ANISOTROPIC_SCALING;
        } else {
            modState = Modification::SCULPTING_SCALING;
        }        
    }
    
    void PAMManifold::changeScalingSingleRib(float scale)
    {
       _scaleFactor = scale;
    }
    
    void PAMManifold::endScalingSingleRib(float scale)
    {
        
        if (modState == Modification::SCULPTING_SCALING)
        {
            modState = Modification::NONE;
            vector<VertexID> allAffectedVerticies;
            for (int i = 0; i < _edges_to_scale.size(); i++) {
                vector<VertexID> affected = change_rib_radius(*this, _edges_to_scale[i], _centroids[i], edgeInfo, 1 + (_scaleFactor - 1)*_scale_weight_vector[i]); //update _manifold
                allAffectedVerticies.insert(allAffectedVerticies.end(), affected.begin(), affected.end());
            }
            
            _scale_weight_vector.clear();
            _edges_to_scale.clear();
            _sculpt_verticies_to_scale.clear();
            updateVertexPositionOnGPU_Vector(allAffectedVerticies);
            updateVertexNormOnGPU_Vector(allAffectedVerticies);
            
        }
        else if (modState == Modification::SCULPTING_ANISOTROPIC_SCALING)
        {
            modState = Modification::NONE;
            vector<VertexID> allAffectedVerticies;
            
            for (int i = 0; i < _edges_to_scale.size(); i ++)
            {
                HalfEdgeID ribID = _edges_to_scale[i];
                float scale =   1 + (_scaleFactor - 1)*_scale_weight_vector[i];
                for (Walker w = walker(ribID); !w.full_circle(); w = w.next().opp().next()) {
                    VertexID vID = w.vertex();
                    Vec3f proj = _anisotropic_projections[vID];
                    Vec newPos = Vec(posf(vID) + (scale - 1) * proj);
                    pos(vID) = newPos;
                    allAffectedVerticies.push_back(vID);
                }
            }
            
            _scale_weight_vector.clear();
            _edges_to_scale.clear();
            updateVertexPositionOnGPU_Vector(allAffectedVerticies);
            updateVertexNormOnGPU_Vector(allAffectedVerticies);
        }
    }
    
#pragma mark - PIN POINT METHODS
    void PAMManifold::createPinPoint(CGLA::Vec3f touchPoint, std::vector<CGLA::Vec3f>& pinPointRib)
    {
        if (modState == Modification::NONE)
        {
            if (!closestVertexID_3D(touchPoint, _pinVertexID)) {
                return;
            }
            
            //find rib
            Walker walker = this->walker(_pinVertexID).opp();
            if (edgeInfo[walker.halfedge()].is_spine()) {
                walker = walker.next();
            }
            assert(edgeInfo[walker.halfedge()].is_rib());
            _pinHalfEdgeID = walker.halfedge();
            
            //draw pin line
            for (Walker ringWalker = this->walker(walker.halfedge());
                 !ringWalker.full_circle();
                 ringWalker = ringWalker.next().opp().next())
            {
                Vec3f p = posf(ringWalker.vertex());
                pinPointRib.push_back(getModelMatrix().mul_3D_point(p));
            }

            modState = Modification::PIN_POINT_SET;
        }
    }
    
    void PAMManifold::deleteCurrentPinPoint()
    {
        modState = Modification::NONE;
    }
#pragma mark - PIVOT POINT METHODS
    bool PAMManifold::createPivotPoint(CGLA::Vec3f touchPoint)
    {
        VertexID vID;
        if(!closestVertexID_3D(touchPoint, vID)) {
            return false;
        }
        
        if (is_pole(*this, vID)) {
            return false;
        }
        
        //find rib edge
        Walker walker = this->walker(vID).opp();
        if (edgeInfo[walker.halfedge()].is_spine()) {
            walker = walker.next().opp();
        }
        assert(edgeInfo[walker.halfedge()].is_rib());
        _pivotHalfEdgeID = walker.halfedge();
        
        return true;
    }
    
    bool PAMManifold::toPivotFromPinDirection(HMesh::HalfEdgeID& toPivothID)
    {
        number_rib_edges(*this, edgeInfo); // number rings
        
        int pivot_loop_id = edgeInfo[_pivotHalfEdgeID].id;
        int pin_loop_id = edgeInfo[_pinHalfEdgeID].id;
        
        if (pivot_loop_id == pin_loop_id) {
            RA_LOG_WARN("[WARNING] Pin is same as pivot rib");
            return false;
        }
        
        //find which way is the pivot
        Walker w1 = this->walker(_pinHalfEdgeID).next();
        Walker w2 = w1.opp().next().opp().next();
        Walker sW1 = w1;
        Walker sW2 = w2;
        
        bool w1ReachedRibJunction = false;
        bool w2ReachedRibJunction = false;
        int iterations = 0;
        while (true) {
            HalfEdgeID hID = w1.next().halfedge();
            HalfEdgeID hID2 = w2.next().halfedge();
            
            if (edgeInfo[hID].id == pivot_loop_id) {
                toPivothID = sW1.halfedge();
                return YES;
            } else if (edgeInfo[hID2].id == pivot_loop_id) {
                toPivothID = sW2.halfedge();
                return YES;
            }
            
            if (edgeInfo[hID].edge_type != RIB_JUNCTION) {
                w1 = w1.next().opp().next();
            } else {
                w1ReachedRibJunction = true;
            }
            
            if (edgeInfo[hID2].edge_type != RIB_JUNCTION) {
                w2 = w2.next().opp().next();
            } else {
                w2ReachedRibJunction = true;
            }
            
            if (w1ReachedRibJunction && w2ReachedRibJunction) {
                RA_LOG_WARN("Pin point is traped betwen to ribJunction");
                return false;
            }
            
            //just in case smth goes wrong
            iterations++;
            if (iterations>1000) {
                return false;
            }
        }
        
        return true;
    }
    
    bool PAMManifold::setTransformedArea()
    {
        HalfEdgeID toPivotEdge;
        bool foundPivotDir = toPivotFromPinDirection(toPivotEdge); //spine towards the pivot
        if (!foundPivotDir) {
            RA_LOG_WARN("Didnt fine direction to pivot");
            return NO;
        }
        
        _transition_verticies.clear();
        _transformed_verticies.clear();
        
        _deformDirHalfEdge = toPivotEdge;
        assert(edgeInfo[toPivotEdge].is_spine());
        Walker pivotDir = this->walker(toPivotEdge);
        
        //    int pin_loop_id = _edgeInfo[_pinHalfEdgeID].id;
        int pivot_loop_id = edgeInfo[_pivotHalfEdgeID].id;
        
        map<VertexID, int> vertexToLoop;
        vector<int> loopsToDeform;
        map<int, float> ringToDeformValue;
        
        //Go through transition area. Stop when pointing to the loop containing pivot.
        //Assigh loopID for every vertex along the way
        HalfEdgeID loopRib = pivotDir.next().halfedge();
        int loopID = edgeInfo[loopRib].id;
        while (loopID != pivot_loop_id) {
            loopsToDeform.push_back(loopID);
            vector<VertexID> verticies = verticies_along_the_rib(*this, loopRib, edgeInfo);
            for (VertexID vID: verticies) {
                vertexToLoop[vID] = loopID;
                _transition_verticies.push_back(vID);
            }
            
            pivotDir = pivotDir.next().opp().next();
            loopRib = pivotDir.next().halfedge();
            loopID = edgeInfo[loopRib].id;
        }
        
        //Get centroid for future rotation
        _centerOfRotation = centroid_for_rib(*this, pivotDir.next().halfedge(), edgeInfo);
        
        //Assign weight deformation for angle to the loops
        for (int i = 0; i < loopsToDeform.size(); i++) {
            int lID = loopsToDeform[i];
            float r = loopsToDeform.size();
            float x = (i+1)/r;
            float weight = pow(pow(x, 2)-1, 2);
            ringToDeformValue[lID] = 1-weight;
        }
        
        _vertexToLoop = vertexToLoop;
        _loopsToDeform = loopsToDeform;
        _ringToDeformValue = ringToDeformValue;
        _transformed_verticies = allVerticiesInDirection(pivotDir);
        changeVerticiesColor_Set(_transformed_verticies, true);
        changeVerticiesColor_Vector(_transition_verticies,Vec4uc(0, 120, 180, 255));
        return YES;
    }
    
    bool PAMManifold::setTransformedAreaForPosingRotate(HMesh::Walker& awayFromTransform)
    {
        
        Walker toTransform = awayFromTransform.opp().next().opp().next();
        number_rib_edges(*this, edgeInfo); // number rings
        assert(edgeInfo[awayFromTransform.halfedge()].is_spine());
        
        map<VertexID, int> vertexToLoop;
        vector<int> loopsToDeform;
        map<int, float> ringToDeformValue;
        
        float dist = 0.2;
        float cur_dist = 0;
        Walker curWalker = awayFromTransform;
        Vec3f lastCentr = centroid_for_rib(*this, _pivotHalfEdgeID, edgeInfo);
        while (cur_dist <= dist && !is_pole(*this, curWalker.vertex()) && !edgeInfo[curWalker.next().halfedge()].is_rib_junction()) {
            HalfEdgeID loopRib = curWalker.next().halfedge();
            int loopID = edgeInfo[loopRib].id;
            loopsToDeform.push_back(loopID);
            
            vector<VertexID> verticies = verticies_along_the_rib(*this, loopRib, edgeInfo);
            for (VertexID vID: verticies) {
                vertexToLoop[vID] = loopID;
                _transition_verticies.push_back(vID);
            }
            
            Vec3f curCentr = centroid_for_rib(*this, loopRib, edgeInfo);
            cur_dist += (curCentr - lastCentr).length();
            lastCentr = curCentr;
            curWalker = curWalker.next().opp().next();
        }
        curWalker = curWalker.next().opp().next();
        _deformDirHalfEdge = curWalker.opp().halfedge();
        std::reverse(loopsToDeform.begin(),loopsToDeform.end());
        
        int loopID = edgeInfo[_pivotHalfEdgeID].id;
        loopsToDeform.push_back(loopID);
        vector<VertexID> verticies = verticies_along_the_rib(*this, _pivotHalfEdgeID, edgeInfo);
        for (VertexID vID: verticies) {
            vertexToLoop[vID] = loopID;
            _transition_verticies.push_back(vID);
        }
        
        cur_dist = 0;
        curWalker = toTransform;
        lastCentr = centroid_for_rib(*this, _pivotHalfEdgeID, edgeInfo);
        while (cur_dist <= dist && !is_pole(*this, curWalker.vertex()) && !edgeInfo[curWalker.next().halfedge()].is_rib_junction()) {
            HalfEdgeID loopRib = curWalker.next().halfedge();
            int loopID = edgeInfo[loopRib].id;
            loopsToDeform.push_back(loopID);
            
            vector<VertexID> verticies = verticies_along_the_rib(*this, loopRib, edgeInfo);
            for (VertexID vID: verticies) {
                vertexToLoop[vID] = loopID;
                _transition_verticies.push_back(vID);
            }
            
            Vec3f curCentr = centroid_for_rib(*this, loopRib, edgeInfo);
            cur_dist += (curCentr - lastCentr).length();
            lastCentr = curCentr;
            curWalker = curWalker.next().opp().next();
        }
        _deformDirHalfEdgeEnd = curWalker.prev().halfedge();
        _transformed_verticies = allVerticiesInDirection(curWalker);
        
        //Get centroid for future rotation
        _centerOfRotation = centroid_for_rib(*this, _pivotHalfEdgeID, edgeInfo);
        
        //Assign weight deformation for angle to the loops
        int s = ceil(loopsToDeform.size()/2);
        for (int i = 0; i < loopsToDeform.size(); i++) {
            int lID = loopsToDeform[i];
            float r = loopsToDeform.size();
            float x = (i+1)/r;
            float weight = pow(pow(x, 2)-1, 2);
            ringToDeformValue[lID] = 1 - weight;
        }
        
        //    for (int i = s; i < loopsToDeform.size(); i++) {
        //        int lID = loopsToDeform[i];
        //        float x = ((i%s)+1)/(float)s;
        //        float weight = pow(pow(x, 2)-1, 2);
        //        ringToDeformValue[lID] = weight;
        //    }
        
        _vertexToLoop = vertexToLoop;
        _loopsToDeform = loopsToDeform;
        _ringToDeformValue = ringToDeformValue;
        
        changeVerticiesColor_Set(_transformed_verticies,true);
        changeVerticiesColor_Vector(_transition_verticies, Vec4uc(0, 120, 180, 255));
        return true;
    }

    
#pragma mark - ROTATING THE BRANCH TREE
    void PAMManifold::startBending(CGLA::Vec3f touchPoint, float angle)
    {
        if (createPivotPoint(touchPoint)) {
            if (setTransformedArea()) {
                _rotAngle = angle;
                modState = Modification::BRANCH_ROTATION;
                _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
            }
        }
    }
    
    void PAMManifold::continueBending(float angle)
    {
        _rotAngle = angle;
    }
    
    void PAMManifold::endBendingWithAngle(float angle)
    {
        if (modState != Modification::BRANCH_ROTATION) {
            return;
        }
        
//        [self saveState];
        
        Vec3f zAxis = invert_affine(viewMatrix).mul_3D_vector(Vec3f(0, 0, -1));
        Mat4x4f toOrigin = translation_Mat4x4f(-1*_centerOfRotation);
        Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*angle);
        Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
        Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
        
        for (VertexID vID: _transformed_verticies) {
            pos(vID) = tMatrix.mul_3D_point(pos(vID));
        }
        
        //deformable area
        map<int,Mat4x4f> rotMatricies;
        for (auto lid: _loopsToDeform) {
            float weight = _ringToDeformValue[lid];
            float angle = weight * _rotAngle;
            Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*angle);
            Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
            rotMatricies[lid]=tMatrix;
        }
        
        for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it)
        {
            VertexID vid = it->first;
            int lid = it->second;
            Mat4x4f tMatrix = rotMatricies[lid];
            pos(vid) = tMatrix.mul_3D_point(pos(vid));
        }
        
        rotateRingsFrom(_deformDirHalfEdge, _pivotHalfEdgeID);
        
        modState = Modification::PIN_POINT_SET;
        
        updateVertexPositionOnGPU_Set(_transformed_verticies);
        updateVertexPositionOnGPU_Vector(_transition_verticies);
        changeVerticiesColor_Set(_transformed_verticies,false);
        changeVerticiesColor_Vector(_transition_verticies,false);
    }
    
    void PAMManifold::rotateRingsFrom(HMesh::HalfEdgeID pivotDirHID, HMesh::HalfEdgeID pivotHalfEdge)
    {
        assert(edgeInfo[pivotDirHID].is_spine());
        assert(edgeInfo[pivotHalfEdge].is_rib());
        
        //Get all centroids
        vector<Vec3f> centroids;
        vector<vector<VertexID>> allVerticies;
        vector<Vec3f> currentNorms;
        vector<Vec3f> desiredNorms;
        vector<Vec3f> movePoints;
        
        Walker pivotDir = this->walker(pivotDirHID);
        
        //Get a centroid before the first one. For better quality
        Vec3f cOpp = centroid_for_rib(*this, pivotDir.opp().next().halfedge(), edgeInfo);
        
        HalfEdgeID loopRib = pivotDir.next().halfedge();
        int loopID = edgeInfo[loopRib].id;
        int pivot_loop_id = edgeInfo[_pivotHalfEdgeID].id;
        while (loopID != pivot_loop_id) {
            Vec3f c = centroid_for_rib(*this, loopRib, edgeInfo);
            centroids.push_back(c);
            vector<VertexID> verticies = verticies_along_the_rib(*this, loopRib, edgeInfo);
            allVerticies.push_back(verticies);
            
            Mat3x3d cov(0);
            for (int i = 0; i < verticies.size(); i++) {
                VertexID vID = verticies[i];
                Vec pos = this->pos(vID);
                Vec d = pos - Vec(c);
                Mat3x3d m;
                outer_product(d,d,m);
                cov += m;
            }
            
            Mat3x3d Q, L;
            int sol = power_eigensolution(cov, Q, L);
            
            Vec3d n;
            assert(sol>=2);
            n = normalize(cross(Q[0],Q[1]));
            currentNorms.push_back(Vec3f(n));
            
            pivotDir = pivotDir.next().opp().next();
            loopRib = pivotDir.next().halfedge();
            loopID = edgeInfo[loopRib].id;
        }
        
        //Get a centroid after the last one. For better quality
        Vec3f cLast = centroid_for_rib(*this, pivotDir.next().halfedge(), edgeInfo);
        
        assert(centroids.size() == currentNorms.size());
        
        if (centroids.size() < 3) {
            RA_LOG_WARN("Deformable area");
            return;
        }
        
        //Get desired norms
        for (int i = 0; i < centroids.size(); i++) {
            Vec3f v1, v2;
            if (i == 0) {
                v1 = centroids[1] - centroids[0];
                v2 = centroids[0] - cOpp;
            } else if (i == (centroids.size() - 1)) {
                v1 = cLast - centroids[i];
                v2 = centroids[i] - centroids[i-1];
            } else {
                v1 = centroids[i] - centroids[i-1];
                v2 = centroids[i+1] - centroids[i];
            }
            Vec3f n = 0.5f*(v1 + v2);
            desiredNorms.push_back(n);
        }
        
        assert(desiredNorms.size() == currentNorms.size());
        
        //Rotate norms
        for (int i = 0; i < centroids.size(); i++) {
            Vec3f axisOfRotation = normalize(cross(currentNorms[i], desiredNorms[i]));
            float angle = signedAngleBetweenReferenceVector3(currentNorms[i], desiredNorms[i]);
            
            if (angle > M_PI_2) {
                angle = -1*(M_PI - angle);
            }
            RA_LOG_INFO("Angle:%f", angle);
            
            Mat4x4f toOrigin = translation_Mat4x4f(-1 * centroids[i]);
            Mat4x4f rotMatrix = rotation_Mat4x4f(axisOfRotation, -1*angle);
            Mat4x4f fromOrigin = translation_Mat4x4f(centroids[i]);
            Mat4x4f tMatrix = (fromOrigin * rotMatrix) * toOrigin;
            
            vector<VertexID> verticies = allVerticies[i];
            for (VertexID vid: verticies) {
                pos(vid) = tMatrix.mul_3D_point(pos(vid));;
            }
        }
    }

#pragma mark - SCALING THE BRANCH TREE
    void PAMManifold::startScalingBranch(CGLA::Vec3f touchPoint, float scale)
    {
        if (createPivotPoint(touchPoint)) {
            if (setTransformedArea()) {
                _scaleFactor = scale;
                modState = Modification::BRANCH_SCALING;
                _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
            }
        }
    }
    
    void PAMManifold::continueScalingBranch(float scale)
    {
        _scaleFactor = scale;
    }
    
    void PAMManifold::endScalingBranchTree(float scale)
    {
        if (modState != Modification::BRANCH_SCALING) {
            return;
        }
        
//        [self saveState];
        
        _scaleFactor = scale;
        
        Mat4x4f toOrigin = translation_Mat4x4f(-1 * _centerOfRotation);
        Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
        Mat4x4f scaleMatrix = scaling_Mat4x4f(Vec3f(_scaleFactor));
        Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
        
        for (VertexID vid: _transformed_verticies) {
            pos(vid) = tMatrix.mul_3D_point(pos(vid));
        }
        
        //deformable area
        map<int,Mat4x4f> scaleMatricies;
        for (auto lid: _loopsToDeform) {
            float weight = _ringToDeformValue[lid];
            float scale = 1 + (_scaleFactor - 1)*weight;
            Mat4x4f sMatrix = scaling_Mat4x4f(Vec3f(scale));
            scaleMatricies[lid]=sMatrix;
        }
        
        for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
            VertexID vid = it->first;
            int lid = it->second;
            
            Mat4x4f scaleMatrix = scaleMatricies[lid];
            Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
            
            pos(vid) = tMatrix.mul_3D_point(pos(vid));
        }
        
        modState = Modification::PIN_POINT_SET;
        
        updateVertexPositionOnGPU_Set(_transformed_verticies);
        updateVertexPositionOnGPU_Vector(_transition_verticies);
        changeVerticiesColor_Set(_transformed_verticies,false);
        changeVerticiesColor_Vector(_transition_verticies,false);
    }
    
#pragma mark - TRANSLATION OF THE BRANCH TREE
    void PAMManifold::startTranslatingBranchTree(CGLA::Vec3f touchPoint, CGLA::Vec3f translation)
    {
        if (createPivotPoint(touchPoint)) {
            if (setTransformedArea()) {
                _translationStart = translation;
                _translationCurrent = translation - _translationStart;
                modState = Modification::BRANCH_TRANSLATION;
                _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
            }
        }
    }
    
    void PAMManifold::continueTranslatingBranchTree(CGLA::Vec3f translation)
    {
        _translationCurrent = translation - _translationStart;
        ;
    }
    
    void PAMManifold::endTranslatingBranchTree(CGLA::Vec3f translation)
    {
        if (modState != Modification::BRANCH_TRANSLATION) {
            return;
        }
        
//        [self saveState];
        
        _translationCurrent = translation - _translationStart;
        
        Mat4x4f translationMatrix = translation_Mat4x4f(_translationCurrent);
        
        for (VertexID vid: _transformed_verticies) {
            pos(vid) = translationMatrix.mul_3D_point(pos(vid));
        }
        
        //deformable area
        map<int,Mat4x4f> scaleMatricies;
        for (auto lid: _loopsToDeform) {
            scaleMatricies[lid]= translation_Mat4x4f(_ringToDeformValue[lid] * _translationCurrent);
        }
        
        for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
            VertexID vid = it->first;
            int lid = it->second;
            pos(vid) = scaleMatricies[lid].mul_3D_point(pos(vid));
        }
        
        modState = Modification::PIN_POINT_SET;
        
        rotateRingsFrom(_deformDirHalfEdge, _pivotHalfEdgeID);
        
        updateVertexPositionOnGPU_Set(_transformed_verticies);
        updateVertexPositionOnGPU_Vector(_transition_verticies);
        changeVerticiesColor_Set(_transformed_verticies,false);
        changeVerticiesColor_Vector(_transition_verticies,false);
    }
    
#pragma mark - POSING ROTATE
    void PAMManifold::startPosingRotate(CGLA::Vec3f touchPoint, float angle)
    {
        if (createPivotPoint(touchPoint)) {
            _transformed_verticies.clear();
            _transition_verticies.clear();
            _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
            set<VertexID> returnSet;
            Walker bigger = biggerSide(_pivotHalfEdgeID,returnSet);
            setTransformedAreaForPosingRotate(bigger);
            modState = Modification::BRANCH_POSE_ROTATE;
        }
    }
    
    void PAMManifold::continuePosingRotate(float angle)
    {
        _rotAngle = angle;
    }
    
    void PAMManifold::endPosingRotate(float angle)
    {
        if (modState != Modification::BRANCH_POSE_ROTATE) {
            return;
        }
        
//        [self saveState];
        
        modState = Modification::NONE;
        
        _rotAngle = angle;
        
        Vec3f zAxis = invert_affine(viewMatrix).mul_3D_vector(Vec3f(0, 0, -1));
        Mat4x4f toOrigin = translation_Mat4x4f(-1*_centerOfRotation);
        Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*angle);
        Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
        Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
        
        for (VertexID vID: _transformed_verticies) {
            pos(vID) = tMatrix.mul_3D_point(pos(vID));
        }
        
        //deformable area
        map<int,Mat4x4f> rotMatricies;
        for (auto lid: _loopsToDeform) {
            float weight = _ringToDeformValue[lid];
            float angle = weight * _rotAngle;
            Mat4x4f rotMatrix = rotation_Mat4x4f(zAxis, -1*angle);
            Mat4x4f tMatrix = fromOrigin * rotMatrix * toOrigin;
            rotMatricies[lid]=tMatrix;
        }
        
        for (auto it = _vertexToLoop.begin(); it!=_vertexToLoop.end(); ++it) {
            VertexID vid = it->first;
            int lid = it->second;
            Mat4x4f tMatrix = rotMatricies[lid];
            pos(vid) = tMatrix.mul_3D_point(pos(vid));
        }
        
        //    [self rotateRingsFrom:_deformDirHalfEdge toRingID:_deformDirHalfEdgeEnd smooth:0];
                 
        updateVertexPositionOnGPU_Set(_transformed_verticies);
        updateVertexNormOnGPU_Set(_transformed_verticies);
        changeVerticiesColor_Set(_transformed_verticies,false);
        
        updateVertexPositionOnGPU_Vector(_transition_verticies);
        updateVertexNormOnGPU_Vector(_transition_verticies);
        changeVerticiesColor_Vector(_transition_verticies,false);
    }
    
    //Checks with direction from to the left or right of the rib loop is smaller
    HMesh::Walker PAMManifold::biggerSide(HMesh::HalfEdgeID ribLoop, std::set<HMesh::VertexID>& returnSet)
    {
        Walker dir = this->walker(ribLoop).next();
        Walker oppDir = dir.opp().next().opp().next();
        
        //Flood rotational area in both ways
        HalfEdgeAttributeVector<EdgeInfo> sEdgeInfo(allocated_halfedges());
        Walker bWalker = walker(dir.next().halfedge()); //Walk along pivot boundary loop
        Walker opp_bWalker = walker(oppDir.next().halfedge()); //Walk along pivot boundary loop
        queue<HalfEdgeID> hq;
        queue<HalfEdgeID> opp_hq;
        
        for (;!bWalker.full_circle(); bWalker = bWalker.next().opp().next()) {
            HalfEdgeID hID = bWalker.next().halfedge();
            HalfEdgeID opp_hID = bWalker.next().opp().halfedge();
            sEdgeInfo[hID] = EdgeInfo(SPINE, 0);
            sEdgeInfo[opp_hID] = EdgeInfo(SPINE, 0);
            hq.push(hID);
        }
        
        for (;!opp_bWalker.full_circle(); opp_bWalker = opp_bWalker.next().opp().next()) {
            HalfEdgeID hID = opp_bWalker.next().halfedge();
            HalfEdgeID opp_hID = opp_bWalker.next().opp().halfedge();
            sEdgeInfo[hID] = EdgeInfo(SPINE, 0);
            sEdgeInfo[opp_hID] = EdgeInfo(SPINE, 0);
            opp_hq.push(hID);
        }
        
        set<VertexID> floodVerticiesSet;
        set<VertexID> opp_floodVerticiesSet;
        
        while(!hq.empty() && !opp_hq.empty())
        {
            HalfEdgeID h = hq.front();
            Walker w = walker(h);
            hq.pop();
            bool is_spine = edgeInfo[h].edge_type == SPINE;
            for (;!w.full_circle(); w=w.circulate_vertex_ccw(),is_spine = !is_spine) {
                if(sEdgeInfo[w.halfedge()].edge_type == UNKNOWN)
                {
                    EdgeInfo ei = is_spine ? EdgeInfo(SPINE,0) : EdgeInfo(RIB,0);
                    
                    floodVerticiesSet.insert(w.vertex());
                    floodVerticiesSet.insert(w.opp().vertex());
                    
                    sEdgeInfo[w.halfedge()] = ei;
                    sEdgeInfo[w.opp().halfedge()] = ei;
                    hq.push(w.opp().halfedge());
                }
            }
            
            h = opp_hq.front();
            w = walker(h);
            opp_hq.pop();
            is_spine = edgeInfo[h].edge_type == SPINE;
            for (;!w.full_circle(); w=w.circulate_vertex_ccw(),is_spine = !is_spine) {
                if(sEdgeInfo[w.halfedge()].edge_type == UNKNOWN)
                {
                    EdgeInfo ei = is_spine ? EdgeInfo(SPINE,0) : EdgeInfo(RIB,0);
                    
                    opp_floodVerticiesSet.insert(w.vertex());
                    opp_floodVerticiesSet.insert(w.opp().vertex());
                    
                    sEdgeInfo[w.halfedge()] = ei;
                    sEdgeInfo[w.opp().halfedge()] = ei;
                    opp_hq.push(w.opp().halfedge());
                }
            }
        }
        
        if (hq.empty()) {
            returnSet = floodVerticiesSet;
            return oppDir;
        } else {
            returnSet = opp_floodVerticiesSet;
            return dir;
        }
    }
    
//#pragma mark - POSING TRANSLATE
//    void PAMManifold::statePosingTranslateWithTouchPoint(CGLA::Vec3f touchPoint, CGLA::Vec3f translation)
//    {
//        if (createPivotPoint(touchPoint))
//        {
//            _transformed_verticies.clear();
//            _transition_verticies.clear();
//            _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
//            _translationStart = translation;
//            _translationCurrent = translation - _translationStart;
//            
//            Walker bigger = biggerSide(_pivotHalfEdgeID,_transformed_verticies);
//            
//            _deformDirHalfEdge = bigger.opp().next().opp().next().halfedge();
//            changeVerticiesColor_Set(_transformed_verticies,true);
//            modState = Modification::BRANCH_POSE_TRANSLATE;
//        }
//    }
//    
//    void PAMManifold::continuePosingTranslate(CGLA::Vec3f translation)
//    {
//        _translationCurrent = translation - _translationStart;
//    }
    
#pragma mark - DELETE/DETACH BRANCH
    bool PAMManifold::detachBranch(CGLA::Vec3f touchPoint)
    {
        if (modState != Modification::PIN_POINT_SET) {
            RA_LOG_WARN("Cant detach. Pin point is not chosen");
            return false;
        }
        
        number_rib_edges(*this, edgeInfo);
        //    if (_edgeInfo[_pinHalfEdgeID].edge_type == RIB_JUNCTION) {
        //        NSLog(@"Can't detach at a branch junction");
        //        [self.delegate displayHint:@"Can't detach at a branch junction"];
        //        return NO;
        //    }
        
//        [self saveState];
        
        //Decide which side to delete
        Walker up = this->walker(_pinVertexID);
        if (!edgeInfo[up.halfedge()].is_spine()) {
            up = up.prev().opp();
        }
        assert(edgeInfo[up.halfedge()].is_spine());
        Walker down = up.prev().opp().prev().opp();
        assert(edgeInfo[down.halfedge()].is_spine());
        assert(up.opp().vertex() == down.opp().vertex());
        
        Vec3f upCentr = centroid_for_rib(*this, up.next().halfedge(),edgeInfo);
        Vec3f downCentr = centroid_for_rib(*this, down.next().halfedge(), edgeInfo);
        Vec3f pinCentr = centroid_for_rib(*this, _pinHalfEdgeID);
        
        Vec3f upVec = upCentr - pinCentr;
        Vec3f downVec = downCentr - pinCentr;
        
        Mat4x4f mvMatrix = getModelViewMatrix();
        Vec3f pinCentroidWorld = mvMatrix.mul_3D_point(pinCentr);
        Vec3f touchPointWorld = viewMatrix.mul_3D_point(touchPoint);

        touchPointWorld[2] = pinCentroidWorld[2];
        Vec3f touchPointModel = invert_affine(mvMatrix).mul_3D_point(touchPointWorld);
        Vec3f touchDir = touchPointModel- pinCentr;
        
        bool deletingBranchFromBody = false;
        Walker toWalker = up;
        if (dot(touchDir, upVec) >= 0) {
            toWalker = up;
        } else if (dot(touchDir, downVec) >= 0) {
            toWalker = down;
        } else {
            RA_LOG_ERROR("Couldn't decide witch side to detach");
//            [self.delegate displayHint:@"Couldn't decide witch side to detach"];
            return false;
        }
        _deletingBranchFromBody = deletingBranchFromBody;
        
        //Start flooding and get all the verticies to delete/move
        _detached_verticies = allVerticiesInDirection(toWalker);
        
        //Save info
        _deleteBodyUpperRibEdge = toWalker.prev().halfedge(); //botton rib
        _deleteDirectionSpineEdge = toWalker.halfedge(); //spine
        _deleteBranchLowerRibEdge = toWalker.next().halfedge(); //top rib
        _deleteBranchSecondRingEdge = toWalker.next().opp().next().next().opp().next().next().halfedge();
        
        //Delete all connecting boundary spine edges
        vector<HalfEdgeID> edgesToDelete;
        for (Walker boundaryW = walker(_deleteBodyUpperRibEdge);
             !boundaryW.full_circle();
             boundaryW = boundaryW.next().opp().next())
        {
            edgesToDelete.push_back(boundaryW.next().halfedge());
        }
        _deleteBranchNumberOfBoundaryRibs = edgesToDelete.size();
        for (HalfEdgeID hID: edgesToDelete) {
            remove_edge(hID);
        }
        
        bufferVertexDataToGPU();
//        [self rebufferWithCleanup:NO bufferData:YES edgeTrace:NO];
        
        changeVerticiesColor_Set(_detached_verticies,true);
        
//        _pinPointLine = nil;
        modState = Modification::BRANCH_DETACHED;
        return true;
    }
    
    bool PAMManifold::deleteBranch(CGLA::Vec3f touchPoint)
    {
        if (modState != Modification::PIN_POINT_SET) {
            RA_LOG_WARN("Cant delete. Pin point is not chosen");
            return false;
        }
        
//        [self saveState];
        
        if (!detachBranch(touchPoint)) {
            return false;
        }

        //Delete verticies
        for (VertexID vID: _detached_verticies) {
            remove_vertex(vID);
        }
        
        Walker boundaryW = walker(_deleteBodyUpperRibEdge);
        float boundaryRadius = rib_radius(*this, _deleteBodyUpperRibEdge, edgeInfo);
        
        if (_deletingBranchFromBody) {
            int numOfEdges;
            closeHole(_deleteBodyUpperRibEdge, numOfEdges);
            Walker bWalkerOuter = walker(boundaryW.opp().halfedge());
            
            vector<VertexID> vertexToSmooth;
            for (int i = 0; i < numOfEdges; i++) {
                vertexToSmooth.push_back(bWalkerOuter.vertex());
                bWalkerOuter = bWalkerOuter.next().opp().next();
            }
            
            traceEdgeInfo();
//            [self rebufferWithCleanup:NO bufferData:NO edgeTrace:YES];
            
            smoothVerticies(vertexToSmooth,1,true,boundaryRadius/3);
            smoothVerticies(vertexToSmooth,1,false,boundaryRadius/3);
        } else {
//            VertexID poleVID = pole_from_hole(*this, boundaryW.halfedge());
            traceEdgeInfo();
//            [self rebufferWithCleanup:NO bufferData:NO edgeTrace:YES];
            //        [self smoothPole:poleVID edgeDepth:3 iter:2];
        }
        
        bufferVertexDataToGPU();
        traceEdgeInfo();
        buildKDTree();
        
//        [self rebufferWithCleanup:YES bufferData:YES edgeTrace:YES];
        deleteCurrentPinPoint();
        
        return true;
    }
    
    bool PAMManifold::moveDetachedBranch(CGLA::Vec3f touchPoint) {
        if (modState != Modification::BRANCH_DETACHED &&
            modState != Modification::BRANCH_DETACHED_AN_MOVED)
        {
            RA_LOG_WARN("Cant move. Branch was not deattached");
            return false;
        }
        
        VertexID touchVID;
        closestVertexID_3D(touchPoint, touchVID);
        //check if you can possible move here
        int numRibSegments = count_rib_segments(*this, edgeInfo, touchVID);
        if (2*numRibSegments + 2 < _deleteBranchNumberOfBoundaryRibs) {
            return false;
        }
        
        _newAttachVertexID = touchVID ;
        Vec touchPos = pos(touchVID);
        Vec normal = HMesh::normal(*this, touchVID);
        
        Vec boundaryCentroid = Vec(centroid_for_boundary_rib(*this, _deleteBranchLowerRibEdge, edgeInfo));
        Vec secondRingCentroid = Vec(centroid_for_rib(*this, _deleteBranchSecondRingEdge, edgeInfo));
        Vec toTouchPos = touchPos - boundaryCentroid;
        secondRingCentroid += toTouchPos;
        for (VertexID vid: _detached_verticies) {
            pos(vid) = pos(vid) + toTouchPos;
        }
        
        Vec currentNorm = secondRingCentroid - touchPos;
        
        CGLA::Quatd q;
        q.make_rot(normalize(currentNorm), normalize(normal));
        
        for (VertexID vid: _detached_verticies) {
            Vec p = pos(vid);
            p -= touchPos;
            p = q.apply(p);
            p += touchPos;
            pos(vid) = p;
        } 
        
        updateVertexPositionOnGPU_Set(_detached_verticies);
        updateVertexNormOnGPU_Set(_detached_verticies);
        
        changeVerticiesColor_Set(_detached_verticies,true);
        modState = Modification::BRANCH_DETACHED_AN_MOVED;

        return true;
    }
    
    bool PAMManifold::attachDetachedBranch() {

        if (modState != Modification::BRANCH_DETACHED &&
            modState != Modification::BRANCH_DETACHED_AN_MOVED)
        {
            RA_LOG_WARN("Cant attach. Havent detached");
            return false;
        }
        
        if (modState == Modification::BRANCH_DETACHED) {
            stitchBranchToBody(_deleteBodyUpperRibEdge, _deleteBranchLowerRibEdge);
            bufferVertexDataToGPU();
            traceEdgeInfo();
            buildKDTree();
        } else if (modState == Modification::BRANCH_DETACHED_AN_MOVED) {
            Walker boundaryW = walker(_deleteBodyUpperRibEdge);
            float boundaryRadius = rib_boundary_radius(*this, _deleteBodyUpperRibEdge, edgeInfo);
            
            int numOfEdges;
            if (_deletingBranchFromBody) {
                closeHole(_deleteBodyUpperRibEdge,numOfEdges);
                //            numOfEdges = 2*numOfEdges;
                Walker bWalkerOuter = walker(boundaryW.opp().halfedge());
                
                vector<VertexID> vertexToSmooth;
                for (int i = 0; i < numOfEdges; i++) {
                    vertexToSmooth.push_back(bWalkerOuter.vertex());
                    bWalkerOuter = bWalkerOuter.next().opp().next();
                }
                
                traceEdgeInfo();
                
                smoothVerticies(vertexToSmooth,20,true,boundaryRadius);
                smoothVerticies(vertexToSmooth,2,false,boundaryRadius);
            } else {
                VertexID poleVID = pole_from_hole(*this, boundaryW.halfedge());
                numOfEdges = valency(*this, poleVID);
                numOfEdges = numOfEdges/2;
                
                //            if (numOfEdges%2 != 0) {
                //                numOfEdges = numOfEdges/2;
                //            } else {
                //                numOfEdges = numOfEdges/2+1;
                //            }
                
                traceEdgeInfo();
                //            [self smoothPole:poleVID edgeDepth:3 iter:2];
            }
            
            VertexID touchedVID = _newAttachVertexID;
            
            //Create new pole
            VertexID newPoleID;
            float bWidth;
            Vec3f holeCenter, holeNorm;
            HalfEdgeID boundaryHalfEdge;
            bool result = createHoleAtVertex(touchedVID,
                                             numOfEdges,
                                             newPoleID,
                                             bWidth,
                                             holeCenter,
                                             holeNorm,
                                             boundaryHalfEdge);
            if (!result) {
                return false;
            }

            HalfEdgeID deleteBranchUpperOppRibEdge = walker(_deleteBranchLowerRibEdge).opp().halfedge();
            stitchBranchToBody(_deleteBranchLowerRibEdge , boundaryHalfEdge);
            
            vector<VertexID> verteciesToSmooth;
            for (Walker w = walker(deleteBranchUpperOppRibEdge); !w.full_circle(); w = w.next().opp().next()) {
                verteciesToSmooth.push_back(w.vertex());
            }
            //        verteciesToSmooth = verticies_along_the_rib(_manifold, deleteBranchUpperOppRibEdge, _edgeInfo);
            smoothVerticies(verteciesToSmooth,10,true,boundaryRadius);
            
            bufferVertexDataToGPU();
            traceEdgeInfo();
            buildKDTree();
        }
        
        _detached_verticies.clear();
        modState = Modification::NONE;
        return true;
    }
    
    void PAMManifold::closeHole(HMesh::HalfEdgeID hID, int& numOfEdges)
    {
        Walker boundaryW = walker(hID);
        while (valency(*this, boundaryW.vertex()) <= 4) {
            boundaryW = boundaryW.next();
        }
        
        Walker bWalker1 = walker(boundaryW.halfedge());
        Walker bWalker2 = walker(boundaryW.next().halfedge());
        
        vector<HalfEdgeID> bEdges1;
        vector<HalfEdgeID> bEdges2;
        while (valency(*this, bWalker2.vertex()) <= 4)
        {
            bEdges1.push_back(bWalker1.halfedge());
            bEdges2.push_back(bWalker2.halfedge());
            bWalker1 = bWalker1.prev();
            bWalker2 = bWalker2.next();
        }
        
        for (int i = 0; i < bEdges1.size() ; i++) {
            BOOL didStich = stitch_boundary_edges(bEdges1[i], bEdges2[i]);
            RA_LOG_INFO("%i", didStich);
            //            assert(didStich);
        }
        
        numOfEdges = bEdges1.size() + 1;
    }
    
#pragma mark - ROTATE DETACHED BRANCH
    bool PAMManifold::startRotateDetachedBranch(float angle)
    {
        if (modState != Modification::BRANCH_DETACHED &&
            modState != Modification::BRANCH_DETACHED_AN_MOVED)
        {
            RA_LOG_WARN("Cant rotate non detached branch");
            return false;
        }
        
        _rotAngle = angle;
        Vec boundaryCentroid = Vec(centroid_for_boundary_rib(*this, _deleteBranchLowerRibEdge, edgeInfo));
        Vec secondRingCentroid = Vec(centroid_for_rib(*this, _deleteBranchSecondRingEdge, edgeInfo));
        Vec boundaryBodyCentroid = Vec(centroid_for_boundary_rib(*this, _deleteBodyUpperRibEdge, edgeInfo));
        Vec currentNorm;
        if (modState == Modification::BRANCH_DETACHED) {
            currentNorm = boundaryCentroid - boundaryBodyCentroid;
            _zRotatePos = boundaryBodyCentroid;
        } else if (modState == Modification::BRANCH_DETACHED_AN_MOVED) {
            Vec touchPos = pos(_newAttachVertexID);
            currentNorm = secondRingCentroid - touchPos;
            _zRotatePos = touchPos;
        }
        _zRotateVec = currentNorm;
        _prevMod = modState;
        _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
        
        modState = Modification::BRANCH_DETACHED_ROTATE;
        return true;
    }
    
    void PAMManifold::continueRotateDetachedBranch(float angle)
    {
        _rotAngle = angle;
    }
    
    void PAMManifold::endRotateDetachedBranch(float angle)
    {
        _rotAngle = angle;
        modState = _prevMod;
        
        CGLA::Quatd q;
        q.make_rot(-_rotAngle, _zRotateVec);
        
        for (VertexID vid: _detached_verticies) {
            Vec p = pos(vid);
            p -= _zRotatePos;
            p = q.apply(p);
            p += _zRotatePos;
            pos(vid) = p;
        }
    }

#pragma mark - CLONING
    bool PAMManifold::copyBranchToBuffer(CGLA::Vec3f touchPoint)
    {
        if (modState != Modification::PIN_POINT_SET) {
            RA_LOG_WARN("Cant copy. Pin point is not chosen");
            return false;
        }
        
        number_rib_edges(*this, edgeInfo);
        if (edgeInfo[_pinHalfEdgeID].edge_type == RIB_JUNCTION) {
            RA_LOG_ERROR("Can't clone at a branch junction");
//            [self.delegate displayHint:@"Can't clone at a branch junction"];
            return false;
        }
        
        //Decide which side to delete
        Walker up = walker(_pinVertexID);
        if (!edgeInfo[up.halfedge()].is_spine()) {
            up = up.prev().opp();
        }
        assert(edgeInfo[up.halfedge()].is_spine());
        Walker down = up.prev().opp().prev().opp();
        assert(edgeInfo[down.halfedge()].is_spine());
        assert(up.opp().vertex() == down.opp().vertex());
        
        
        Vec3f upCentr = centroid_for_rib(*this, up.next().halfedge(), edgeInfo);
        Vec3f downCentr = centroid_for_rib(*this, down.next().halfedge(), edgeInfo);
        Vec3f pinCentr = centroid_for_rib(*this, _pinHalfEdgeID);
        
        Vec3f upVec = upCentr - pinCentr;
        Vec3f downVec = downCentr - pinCentr;
        
        Mat4x4f mvMatrix = getModelViewMatrix();

        Vec3f pinCentroidWorld = mvMatrix.mul_3D_point(pinCentr);
        Vec3f touchPointWorld = viewMatrix.mul_3D_point(touchPoint);
        
        touchPointWorld[2] = pinCentroidWorld[2];
        Vec3f touchPointModel = invert_affine(mvMatrix).mul_3D_point(touchPointWorld);
        Vec3f touchDir = touchPointModel - pinCentr;
        
        Walker toWalker = up;
        if (dot(touchDir, upVec) >= 0) {
            toWalker = up;
        } else if (dot(touchDir, downVec) >= 0) {
            toWalker = down;
        } else {
            RA_LOG_ERROR("Couldn't decide witch side to detach");
//            [self.delegate displayHint:@"Couldn't decide witch side to detach"];
            return NO;
        }
        
        _cloningDirection = toWalker.halfedge();
        _cloningBodyUpperRibEdge = toWalker.prev().halfedge();
        
        //Number of rib segments araound boundary rib
        _copyNumBoundaryRibSegments = 0;
        for (Walker boundaryW = walker(_cloningBodyUpperRibEdge);
             !boundaryW.full_circle();
             boundaryW = boundaryW.next().opp().next())
        {
            _copyNumBoundaryRibSegments++;
        }
        
        //find all verticies that we need to delete
        _original_verticies_copied =  allVerticiesInDirection(toWalker);
        set<VertexID> all_verticies_to_delete =  allVerticiesInDirection(toWalker.opp());
        //        [self changeVerticiesColor:all_verticies_to_delete toSelected:YES];
        
        //remove verticies from duplicate manifold, so that only copy part is left
        Manifold copyMani = Manifold(*this);
        for (VertexID vID: all_verticies_to_delete) {
            copyMani.remove_vertex(vID);
        }
        copyMani.cleanup();
        
        _copyFaces.clear();
        _copyIndices.clear();
        _copyVerticies.clear();
        extractFromManifold(copyMani,_copyVerticies,_copyFaces,_copyIndices);
        
        changeVerticiesColor_Set(_original_verticies_copied,Vec4uc(250, 89, 14, 255));
        
        //    _pinPointLine = nil;
        modState = Modification::BRANCH_COPIED_BRANCH_FOR_CLONING;
        
        return true;
    }
    
    bool PAMManifold::cloneBranchTo(CGLA::Vec3f touchPoint)
    {
        if (modState != Modification::BRANCH_COPIED_BRANCH_FOR_CLONING) {
            RA_LOG_WARN("No branch was chosen for cloning");
            return false;
        }

        closestVertexID_3D(touchPoint, _newClonedVertexID);
        Vec touchPos = pos(_newClonedVertexID);
        Vec normal = HMesh::normal(*this, _newClonedVertexID);
        
        //check if you can possible move here
        int numRibSegments = count_rib_segments(*this, edgeInfo, _newClonedVertexID);
        if (numRibSegments - 2 < _copyNumBoundaryRibSegments) {
            return false;
        }
        
//        [self saveState];
        
        assert(_copyVerticies.size() != 0);
        assert(_copyFaces.size() != 0);
        assert(_copyIndices.size() != 0);
        
        FaceIDIterator lastFace = faces_end();
        build(_copyVerticies.size(),
              reinterpret_cast<float*>(&_copyVerticies[0]),
              _copyFaces.size(),
              &_copyFaces[0],
              &_copyIndices[0]);
        
        lastFace++;
        Walker w = walker(*lastFace);
        
        vector<HalfEdgeID>newEdges;
        _cloned_verticies.clear();
        allVerticiesAndHalfEdges(_cloned_verticies,newEdges,w.vertex());
        
        Vec boundaryCentroid = Vec(centroid_for_rib(*this, _cloningBodyUpperRibEdge, edgeInfo));
        Vec toTouchPos = touchPos - boundaryCentroid;
        for (VertexID vid: _cloned_verticies) {
            pos(vid) += toTouchPos;
        }
        
        if (!boundaryHalfEdgeForClonedMesh(_cloningBranchLowerRibEdge,newEdges)) {
//            [self undo];
            return false;
        }
        
        vector<VertexID> verticies;
        for(HalfEdgeID hid: newEdges)
        {
            Walker w = walker(hid);
            verticies.push_back(w.vertex());
        }
        
        //    [self rebufferWithCleanup:NO bufferData:YES edgeTrace:NO];
        //    [self changeWireFrameColor:_cloned_verticies toSelected:YES];
        //    return NO;
        
        Walker toSecondRing = walker(_cloningBranchLowerRibEdge);
        toSecondRing = toSecondRing.next().next().opp().next().next().opp();
        _cloningSecondRing = toSecondRing.halfedge();
        Vec secondRingCentroid = Vec(centroid_for_rib(*this, _cloningSecondRing));
        
        Vec currentNorm = secondRingCentroid - touchPos;
        
        CGLA::Quatd q;
        q.make_rot(normalize(currentNorm), normalize(normal));
        
        for (VertexID vid: _cloned_verticies) {
            Vec p = pos(vid);
            p -= touchPos;
            p = q.apply(p);
            p += touchPos;
            pos(vid) = p;
        }
        
        bufferVertexDataToGPU();
//        [self rebufferWithCleanup:NO bufferData:YES edgeTrace:NO];
        changeVerticiesColor_Vector(_cloned_verticies,Vec4uc(0,200,0,255));
        changeVerticiesColor_Set(_original_verticies_copied,Vec4uc(250, 89, 14,255));
        
        modState = Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE;
        
        return true;
    }
    
    bool PAMManifold::attachClonedBranch()
    {
        if (modState != Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE)
        {
            RA_LOG_WARN("Cant attach. Havent cloned");
            return false;
        }
        
        //Create new pole
        VertexID newPoleID;
        float bWidth;
        Vec3f holeCenter, holeNorm;
        HalfEdgeID boundaryHalfEdge;
        BOOL result = createHoleAtVertex(_newClonedVertexID,
                                         _copyNumBoundaryRibSegments/2,
                                         newPoleID,
                                         bWidth,
                                         holeCenter,
                                         holeNorm,
                                         boundaryHalfEdge);
        if (!result) {
            return false;
        }
        
        HalfEdgeID cloneBranchUpperOppRibEdge = walker(_cloningBranchLowerRibEdge).opp().halfedge();
        stitchBranchToBody(_cloningBranchLowerRibEdge,boundaryHalfEdge);
        
        vector<VertexID> verteciesToSmooth;
        for (Walker w = walker(cloneBranchUpperOppRibEdge); !w.full_circle(); w = w.next().opp().next()) {
            verteciesToSmooth.push_back(w.vertex());
        }
        
        if (!PAMMesh::PAMSettingsManager::getInstance().spineSmoothing) {
            smoothVerticies(verteciesToSmooth,2,false,0.1);
        } else {
            traceEdgeInfo();
            smoothVerticies(verteciesToSmooth,10,true,0.1);
        }
        
        bufferVertexDataToGPU();
        traceEdgeInfo();
        buildKDTree();

        changeVerticiesColor_Set(_original_verticies_copied,Vec4uc(250, 89, 14, 255));
        _cloned_verticies.clear();
        modState = Modification::BRANCH_COPIED_BRANCH_FOR_CLONING;

        return true;
    }
    
    void PAMManifold::dismissCopiedBranch()
    {
        modState = Modification::NONE;
//        _pinPointLine = nil;
        changeVerticiesColor_Set(_original_verticies_copied,false);
    }
    
    void PAMManifold::extractFromManifold(HMesh::Manifold& mani,
                                          std::vector<CGLA::Vec3f>& verticies,
                                          std::vector<int>& faces,
                                          std::vector<int>&  indices)
    {
        for (VertexIDIterator vid = mani.vertices_begin(); vid != mani.vertices_end(); ++vid) {
            assert((*vid).index < mani.no_vertices());
            Vec3f position = mani.posf(*vid);
            verticies.push_back(position);
        }
        
        for (FaceIDIterator fid = mani.faces_begin(); fid != mani.faces_end(); ++fid) {
            //iterate over every vertex of the face
            int vertexNum = 0;
            for(Walker w = mani.walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
                unsigned int index = vID.index;
                assert(index < mani.no_vertices());
                vertexNum++;
                indices.push_back(index);
            }
            faces.push_back(vertexNum);
        }
    }
    
#pragma mark - SCALE CLONED BRANCH
    bool PAMManifold::startScaleClonedBranch(float scale)
    {
        if (modState != Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE)
        {
            RA_LOG_WARN("Cant rotate non cloned branch");
            return false;
        }
        
        _scaleFactor = scale;
        
        _centerOfRotation = posf(_newClonedVertexID);
        
        _prevMod = modState;
        _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
        
        modState = Modification::BRANCH_CLONE_SCALING;
        return true;
    }
    
    void PAMManifold::continueScaleClonedBranch(float scale)
    {
        _scaleFactor = scale;
    }
    
    void PAMManifold::endScaleClonedBranch(float scale)
    {
        _scaleFactor = scale;
        modState = _prevMod;
        
        Mat4x4f toOrigin = translation_Mat4x4f(-1 * _centerOfRotation);
        Mat4x4f fromOrigin = translation_Mat4x4f(_centerOfRotation);
        Mat4x4f scaleMatrix = scaling_Mat4x4f(Vec3f(_scaleFactor));
        Mat4x4f tMatrix = (fromOrigin * scaleMatrix) * toOrigin;
        
        for (VertexID vid: _cloned_verticies) {
            pos(vid) = tMatrix.mul_3D_point(pos(vid));
        }
        modState = Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE;
    }

#pragma mark - ROTATE CLONED BRANCH
    bool PAMManifold::startRotateClonedBranch(float angle)
    {
        if (modState != Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE)
        {
            RA_LOG_WARN("Cant rotate non cloned branch");
            return false;
        }
        
        _rotAngle = angle;
        
        Vec secondRingCentroid = Vec(centroid_for_rib(*this, _cloningSecondRing));
        Vec touchPos = pos(_newClonedVertexID);
        _zRotatePos = touchPos;
        _zRotateVec = secondRingCentroid - touchPos;;
        _prevMod = modState;
        _current_rot_position = VertexAttributeVector<Vec3f>(no_vertices());
        
        modState = Modification::BRANCH_CLONE_ROTATION;
        return true;
    }
    
    void PAMManifold::continueRotateClonedBranch(float angle)
    {
        _rotAngle = angle;
    }
    
    void PAMManifold::endRotateClonedBranch(float angle)
    {
        _rotAngle = angle;
        modState = _prevMod;
        
        CGLA::Quatd q;
        q.make_rot(-_rotAngle, _zRotateVec);
        
        for (VertexID vid: _cloned_verticies) {
            Vec p = pos(vid);
            p -= _zRotatePos;
            p = q.apply(p);
            p += _zRotatePos;
            pos(vid) = p;
        }
        modState = Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE;
    }
    
#pragma mark -  UPDATE GPU DATA
    
    void PAMManifold::updateVertexPositionOnGPU_Vector(std::vector<HMesh::VertexID>& verticies)
    {
        positionDataBuffer->bind();
        unsigned char* tempVerticies = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: verticies) {
            Vec3f pf = posf(vid);
            int index = vertexIDtoIndex[vid];
            memcpy(tempVerticies + index*sizeof(Vec3f), pf.get(), sizeof(Vec3f));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PAMManifold::updateVertexNormOnGPU_Vector(std::vector<HMesh::VertexID>& verticies)
    {
        normalDataBuffer->bind();
        unsigned char* tempNormal = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: verticies) {
            Vec3f normalf = HMesh::normalf(*this, vid);
            int index = vertexIDtoIndex[vid];
            memcpy(tempNormal + index*sizeof(Vec3f), normalf.get(), sizeof(Vec3f));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PAMManifold::updateVertexPositionOnGPU_Set(std::set<HMesh::VertexID>& verticies)
    {
        positionDataBuffer->bind();
        unsigned char* tempVerticies = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: verticies) {
            Vec3f pf = posf(vid);
            int index = vertexIDtoIndex[vid];
           memcpy(tempVerticies + index*sizeof(Vec3f), pf.get(), sizeof(Vec3f));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PAMManifold::updateVertexNormOnGPU_Set(std::set<HMesh::VertexID>& verticies)
    {
        normalDataBuffer->bind();
        unsigned char* tempNormal = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: verticies) {
            Vec3f normalf = HMesh::normalf(*this, vid);
            int index = vertexIDtoIndex[vid];
            memcpy(tempNormal + index*sizeof(Vec3f), normalf.get(), sizeof(Vec3f));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PAMManifold::changeVerticiesColor_Vector(std::vector<HMesh::VertexID>& vertecies, CGLA::Vec4uc selectColor)
    {
        colorDataBuffer->bind();
        unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: vertecies) {
            int index = vertexIDtoIndex[vid];
            memcpy(temp + index*sizeof(Vec4uc), selectColor.get(), sizeof(Vec4uc));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        
    }
    
    void PAMManifold::changeVerticiesColor_Set(std::set<HMesh::VertexID>& vertecies, CGLA::Vec4uc selectColor)
    {
        colorDataBuffer->bind();
        unsigned char* temp = (unsigned char*) glMapBufferOES(GL_ARRAY_BUFFER, GL_WRITE_ONLY_OES);
        for (VertexID vid: vertecies) {
            int index = vertexIDtoIndex[vid];
            memcpy(temp + index*sizeof(Vec4uc), selectColor.get(), sizeof(Vec4uc));
        }
        glUnmapBufferOES(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PAMManifold::changeVerticiesColor_Vector(std::vector<HMesh::VertexID>& vertecies, bool isSelected)
    {
        Vec4uc selectColor;
        if (isSelected) {
            selectColor = Vec4uc(240, 0, 0, 255);
        } else {
            selectColor = Vec4uc(200,200,200,255);
        }
        changeVerticiesColor_Vector(vertecies,selectColor);
    }
    
    void PAMManifold::changeVerticiesColor_Set(std::set<HMesh::VertexID>& vertecies, bool isSelected)
    {
        Vec4uc selectColor;
        if (isSelected) {
            selectColor = Vec4uc(240, 0, 0, 255);
        } else {
            selectColor = Vec4uc(200,200,200,255);
        }
        changeVerticiesColor_Set(vertecies,selectColor);
    }
    
#pragma mark - DELETE
    float PAMManifold::signedAngleBetweenReferenceVector3(CGLA::Vec3f refVector, CGLA::Vec3f vector)
    {
        float l = length(refVector)*length(vector);
        Vec3f norm = cross(refVector, vector);
        float cosa = dot(refVector, vector)/l;
        float sina = length(cross(refVector, vector))/l;
        
        float angle = atan2(sina, cosa);
        float sign = dot(norm, cross(refVector, vector));
        
        if (sign < 0) {
            angle = -angle;
        }
        
        return angle;
    }
}

