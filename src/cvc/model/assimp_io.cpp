/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

// assimp_io — the Assimp-backed cvc::model / cvc::geometry importer. Assimp
// decodes a broad set of mesh/scene formats (obj, ply, stl, fbx, gltf/glb, dae,
// ...) into aiScene; here we lower each aiMesh into a cvc::geometry (positions,
// normals, vertex colors, UV0, tangents) and each aiMaterial into a
// cvc::material, resolving + decoding the base-color texture into a cvc::image
// (below the VTK line). Two handlers are exposed: a model_file_io that returns
// the full multi-mesh scene, and a geometry_file_io that flattens it via
// model::merged() so read_geometry("foo.obj") works. Everything Assimp-specific
// is guarded by CVC_ENABLE_ASSIMP; when it is off the register hooks are no-ops
// and read_model/read_geometry raise "no handler" (mirroring the ImageMagick
// image handler).

#include <algorithm> // read_model_from_memory: the extension, lowercased
#include <cctype>
#include <cvc/core/app.h>
#include <cvc/model/model_file_io.h>
#include <cvc/volume/io_handlers.h>
#include <string>

#ifdef CVC_ENABLE_ASSIMP
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/texture.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/image/image.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace cvc {
namespace {

// Assimp's common importable set (leading dot, matching the geometry_file_io
// handler-map key convention).
model_file_io::extension_list assimp_extensions() {
  model_file_io::extension_list e;
  e.push_back(".obj");
  e.push_back(".ply");
  e.push_back(".stl");
  e.push_back(".fbx");
  e.push_back(".gltf");
  e.push_back(".glb");
  e.push_back(".dae");
  e.push_back(".3ds");
  e.push_back(".blend");
  e.push_back(".x");
  e.push_back(".off");
  e.push_back(".lwo");
  e.push_back(".ms3d");
  e.push_back(".ase");
  e.push_back(".ifc");
  return e;
}

// Post-process flags. NB: no aiProcess_FlipUVs — the GeometryNode flips V in the
// texture path, so UVs are kept exactly as authored.
unsigned int import_flags() {
  return aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace |
         aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality;
}

// model_read_options::exact: what the file stores, in its order, nothing joined or generated.
unsigned int import_flags(const model_read_options &o) { return o.exact ? 0u : import_flags(); }

// The file in its own axes and units. Assimp's Collada and FBX importers turn a Z-up file to Y-up
// (and the Collada one scales it to metres) by folding a transform into the root node, which
// model::nodes now applies; a model is in the file's authoring space (model::extents), and its
// readers turn it themselves (a glTF file's +Y up, a fit's up axis). The file's own node
// transforms are kept.
void keep_file_axes(Assimp::Importer &imp) {
  imp.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  imp.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE, true);
  imp.SetPropertyBool(AI_CONFIG_IMPORT_FBX_IGNORE_UP_DIRECTION, true);
}

// Directory containing `p` ("." if none), for resolving relative texture paths.
std::string dir_of(const std::string &p) {
  std::string::size_type s = p.find_last_of("/\\");
  return s == std::string::npos ? std::string(".") : p.substr(0, s);
}

bool is_absolute_path(const std::string &p) {
  if (p.empty())
    return false;
  if (p[0] == '/' || p[0] == '\\')
    return true;
  // Windows drive letter (e.g. C:\...)
  if (p.size() >= 2 && p[1] == ':')
    return true;
  return false;
}

std::string default_temp_dir() {
  const char *env[] = {std::getenv("TMPDIR"), std::getenv("TMP"), std::getenv("TEMP")};
  for (int i = 0; i < 3; ++i)
    if (env[i] && env[i][0])
      return std::string(env[i]);
  return "/tmp";
}

// An app's embedded-texture temp names: a random tag of its own (two apps of one process) and a
// count (one app's threads).
struct embedded_texture_names {
  static constexpr const char *app_slot = "cvc.model.embedded_texture_names";
  const std::string tag = [] {
    std::random_device rd;
    char t[17];
    std::snprintf(t, sizeof t, "%016llx",
                  (static_cast<unsigned long long>(rd()) << 32) ^
                      static_cast<unsigned long long>(rd()));
    return std::string(t);
  }();
  std::atomic<unsigned long long> counter{0};
};

// Decode an embedded, compressed texture blob (mHeight == 0: pcData is `mWidth`
// bytes of e.g. PNG/JPEG). cvc's image reader only reads from a path, so spill
// the blob to a temp file with the format-hint extension and read_image() it
// with ctx's image handlers.
image decode_embedded_compressed(app &ctx, const aiTexture *tex) {
  std::string hint(tex->achFormatHint);
  std::string ext = hint.empty() ? std::string("bin") : hint;
  // Unique temp name: the app's counter (its threads) and random tag (other apps of this
  // process), qualified by the pid (two processes sharing TMPDIR).
  embedded_texture_names &names = ctx.slot<embedded_texture_names>();
#if defined(_WIN32)
  const long pid = static_cast<long>(_getpid());
#else
  const long pid = static_cast<long>(::getpid());
#endif
  std::string tmp = default_temp_dir() + "/cvc_embedded_tex_" + std::to_string(pid) + "_" +
                    names.tag + "_" + std::to_string(names.counter.fetch_add(1)) + "." + ext;
  {
    std::ofstream ofs(tmp.c_str(), std::ios::binary);
    if (!ofs)
      throw std::runtime_error("could not open temp file for embedded texture");
    ofs.write(reinterpret_cast<const char *>(tex->pcData),
              static_cast<std::streamsize>(tex->mWidth));
  }
  image out;
  try {
    out = read_image(ctx, tmp);
  } catch (...) {
    std::remove(tmp.c_str());
    throw;
  }
  std::remove(tmp.c_str());
  return out;
}

// Build a cvc::image from a raw, uncompressed embedded texture (mHeight != 0:
// pcData is mWidth*mHeight aiTexel, which are BGRA in memory). Convert to cvc's
// interleaved RGBA.
image decode_embedded_raw(const aiTexture *tex) {
  int w = static_cast<int>(tex->mWidth);
  int h = static_cast<int>(tex->mHeight);
  image out(w, h, image::pixel_format::RGBA, image::data_type::u8);
  if (w <= 0 || h <= 0)
    return out;
  unsigned char *d = out.data();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  for (std::size_t i = 0; i < n; ++i) {
    const aiTexel &t = tex->pcData[i];
    d[i * 4 + 0] = t.r;
    d[i * 4 + 1] = t.g;
    d[i * 4 + 2] = t.b;
    d[i * 4 + 3] = t.a;
  }
  return out;
}

// Resolve + load a base-color texture into a cvc::image. A missing/unreadable
// texture is non-fatal: return an empty image (the path is still recorded). ctx's image
// handlers read it.
image load_texture(app &ctx, const aiScene *scene, const std::string &texpath,
                   const std::string &model_dir) {
  if (texpath.empty())
    return image();
  try {
    if (texpath[0] == '*') { // assimp embedded-texture index, e.g. "*0"
      int idx = std::atoi(texpath.c_str() + 1);
      if (idx < 0 || static_cast<unsigned int>(idx) >= scene->mNumTextures)
        return image();
      const aiTexture *tex = scene->mTextures[idx];
      if (!tex)
        return image();
      return tex->mHeight == 0 ? decode_embedded_compressed(ctx, tex) : decode_embedded_raw(tex);
    }
    std::string resolved = is_absolute_path(texpath) ? texpath : (model_dir + "/" + texpath);
    return read_image(ctx, resolved);
  } catch (const std::exception &e) {
    // Don't fail the whole load for a missing/unsupported texture.
    std::cerr << "cvc::model assimp: could not load texture '" << texpath << "': " << e.what()
              << std::endl;
    return image();
  }
}

// Lower one aiMesh into a cvc::geometry.
geometry build_geometry(const aiMesh *mesh) {
  geometry geom;

  const unsigned int nv = mesh->mNumVertices;

  {
    geometry::points_t &pts = geom.points();
    pts.reserve(nv);
    for (unsigned int v = 0; v < nv; ++v) {
      point_t p;
      p[0] = mesh->mVertices[v].x;
      p[1] = mesh->mVertices[v].y;
      p[2] = mesh->mVertices[v].z;
      pts.push_back(p);
    }
  }

  if (mesh->HasNormals()) {
    geometry::normals_t &nrm = geom.normals();
    nrm.reserve(nv);
    for (unsigned int v = 0; v < nv; ++v) {
      geometry::normal_t n;
      n[0] = mesh->mNormals[v].x;
      n[1] = mesh->mNormals[v].y;
      n[2] = mesh->mNormals[v].z;
      nrm.push_back(n);
    }
  }

  if (mesh->HasVertexColors(0)) {
    geometry::colors_t &col = geom.colors();
    col.reserve(nv);
    for (unsigned int v = 0; v < nv; ++v) {
      color_t c;
      c[0] = mesh->mColors[0][v].r;
      c[1] = mesh->mColors[0][v].g;
      c[2] = mesh->mColors[0][v].b;
      col.push_back(c);
    }
  }

  if (mesh->HasTextureCoords(0)) {
    geometry::uvs_t &uv = geom.uvs();
    uv.reserve(nv);
    for (unsigned int v = 0; v < nv; ++v) {
      uv_t t;
      // Keep UVs as authored (no V flip); the texture path handles orientation.
      t[0] = mesh->mTextureCoords[0][v].x;
      t[1] = mesh->mTextureCoords[0][v].y;
      uv.push_back(t);
    }
  }

  if (mesh->HasTangentsAndBitangents()) {
    // Assimp stores a 3-component tangent + a separate bitangent. cvc's tangent
    // is xyz + a handedness w = sign(dot(cross(n, t), bitangent)); reconstruct it.
    geometry::tangents_t &tng = geom.tangents();
    tng.reserve(nv);
    const bool have_n = mesh->HasNormals();
    for (unsigned int v = 0; v < nv; ++v) {
      const aiVector3D &t = mesh->mTangents[v];
      const aiVector3D &b = mesh->mBitangents[v];
      double w = 1.0;
      if (have_n) {
        const aiVector3D &n = mesh->mNormals[v];
        // cross(n, t)
        double cx = n.y * t.z - n.z * t.y;
        double cy = n.z * t.x - n.x * t.z;
        double cz = n.x * t.y - n.y * t.x;
        double dot = cx * b.x + cy * b.y + cz * b.z;
        w = dot < 0.0 ? -1.0 : 1.0;
      }
      tangent_t out;
      out[0] = t.x;
      out[1] = t.y;
      out[2] = t.z;
      out[3] = w;
      tng.push_back(out);
    }
  }

  {
    geometry::tris_t &tris = geom.tris();
    tris.reserve(mesh->mNumFaces);
    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace &face = mesh->mFaces[f];
      // A line's segment is kept (an exact read: needles, a centreline); anything else that is
      // not a triangle is skipped.
      if (face.mNumIndices == 2) {
        line_t ln;
        ln[0] = face.mIndices[0];
        ln[1] = face.mIndices[1];
        geom.lines().push_back(ln);
        continue;
      }
      if (face.mNumIndices != 3)
        continue;
      tri_t tr;
      tr[0] = face.mIndices[0];
      tr[1] = face.mIndices[1];
      tr[2] = face.mIndices[2];
      tris.push_back(tr);
    }
  }

  return geom;
}

// Lower one aiMaterial into a cvc::material (resolving + loading its texture).
material build_material(app &ctx, const aiScene *scene, const aiMaterial *aim,
                        const std::string &model_dir) {
  material mat;

  aiString nm;
  if (aim->Get(AI_MATKEY_NAME, nm) == AI_SUCCESS)
    mat.name = nm.C_Str();

  // base color: prefer the PBR base-color factor, fall back to diffuse.
  aiColor4D base;
  if (aim->Get(AI_MATKEY_BASE_COLOR, base) == AI_SUCCESS) {
    mat.base_color[0] = base.r;
    mat.base_color[1] = base.g;
    mat.base_color[2] = base.b;
    mat.base_color[3] = base.a;
  } else {
    aiColor3D diff(1.f, 1.f, 1.f);
    if (aim->Get(AI_MATKEY_COLOR_DIFFUSE, diff) == AI_SUCCESS) {
      mat.base_color[0] = diff.r;
      mat.base_color[1] = diff.g;
      mat.base_color[2] = diff.b;
    }
  }
  // opacity folds into the base-color alpha.
  float opacity = 1.f;
  if (aim->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS)
    mat.base_color[3] = opacity;

  float mf = 0.f;
  if (aim->Get(AI_MATKEY_METALLIC_FACTOR, mf) == AI_SUCCESS)
    mat.metallic = mf;
  float rf = 1.f;
  if (aim->Get(AI_MATKEY_ROUGHNESS_FACTOR, rf) == AI_SUCCESS)
    mat.roughness = rf;

  aiColor3D em(0.f, 0.f, 0.f);
  if (aim->Get(AI_MATKEY_COLOR_EMISSIVE, em) == AI_SUCCESS) {
    mat.emissive[0] = em.r;
    mat.emissive[1] = em.g;
    mat.emissive[2] = em.b;
  }

  // base-color texture: prefer BASE_COLOR, fall back to legacy DIFFUSE.
  aiString texpath;
  if (aim->GetTexture(aiTextureType_BASE_COLOR, 0, &texpath) == AI_SUCCESS ||
      aim->GetTexture(aiTextureType_DIFFUSE, 0, &texpath) == AI_SUCCESS) {
    mat.base_color_texture_path = texpath.C_Str();
    mat.base_color_texture = load_texture(ctx, scene, mat.base_color_texture_path, model_dir);
  }

  return mat;
}

// The scene's node tree as model::nodes: the root first, then each node's children together, in
// their order (instances() walks the children lists, so the drawing order is depth first either
// way). An explicit stack, since a file's tree can be deep. A mesh index past the scene's meshes is
// dropped.
void read_nodes(const aiScene *s, model &m) {
  const auto add = [&](const aiNode *an) {
    model::node nd;
    nd.name = an->mName.C_Str();
    const aiMatrix4x4 &t = an->mTransformation;
    nd.transform = {{t.a1, t.a2, t.a3, t.a4, t.b1, t.b2, t.b3, t.b4, t.c1, t.c2, t.c3, t.c4, t.d1,
                     t.d2, t.d3, t.d4}};
    for (unsigned int i = 0; i < an->mNumMeshes; ++i)
      if (an->mMeshes[i] < s->mNumMeshes)
        nd.meshes.push_back(static_cast<int>(an->mMeshes[i]));
    m.nodes.push_back(nd);
    return static_cast<int>(m.nodes.size() - 1);
  };
  std::vector<std::pair<const aiNode *, int>> stack;
  stack.push_back(std::make_pair(s->mRootNode, add(s->mRootNode)));
  while (!stack.empty()) {
    const aiNode *an = stack.back().first;
    const int index = stack.back().second;
    stack.pop_back();
    for (unsigned int c = 0; c < an->mNumChildren; ++c) {
      if (!an->mChildren[c])
        continue;
      const int child = add(an->mChildren[c]);
      m.nodes[index].children.push_back(child);
      stack.push_back(std::make_pair(an->mChildren[c], child));
    }
  }
}

// The first node named `name`, -1 for none.
int node_named(const model &m, const std::string &name) {
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    if (m.nodes[i].name == name)
      return static_cast<int>(i);
  return -1;
}

model::transform_t transform_of(const aiMatrix4x4 &t) {
  return {{t.a1, t.a2, t.a3, t.a4, t.b1, t.b2, t.b3, t.b4, t.c1, t.c2, t.c3, t.c4, t.d1, t.d2, t.d3,
           t.d4}};
}

// A mesh's bones as its skin: each bone's node (by name) and offset, and each vertex's
// influences, its weights made to sum to 1.
void read_skin(const aiMesh *am, const model &m, model::mesh &mm) {
  if (!am->HasBones())
    return;
  const unsigned int nv = am->mNumVertices;
  std::vector<std::vector<std::pair<int, double>>> per(nv);
  for (unsigned int b = 0; b < am->mNumBones; ++b) {
    const aiBone *ab = am->mBones[b];
    if (!ab)
      continue;
    model::bone bn;
    bn.node = node_named(m, ab->mName.C_Str());
    bn.offset = transform_of(ab->mOffsetMatrix);
    const int index = static_cast<int>(mm.bones.bones.size());
    mm.bones.bones.push_back(bn);
    for (unsigned int w = 0; w < ab->mNumWeights; ++w)
      if (ab->mWeights[w].mVertexId < nv && ab->mWeights[w].mWeight > 0.0f)
        per[ab->mWeights[w].mVertexId].emplace_back(index, ab->mWeights[w].mWeight);
  }
  mm.bones.first.reserve(nv + 1);
  for (unsigned int v = 0; v < nv; ++v) {
    mm.bones.first.push_back(static_cast<boost::uint32_t>(mm.bones.influences.size()));
    double sum = 0.0;
    for (const auto &bw : per[v])
      sum += bw.second;
    for (const auto &bw : per[v])
      mm.bones.influences.emplace_back(bw.first, sum > 0.0 ? bw.second / sum : bw.second);
  }
  mm.bones.first.push_back(static_cast<boost::uint32_t>(mm.bones.influences.size()));
}

// A mesh's anim meshes as its morph targets: each the difference of its positions (and normals)
// from the mesh's, and its weight at rest.
void read_morphs(const aiMesh *am, model::mesh &mm) {
  const unsigned int nv = am->mNumVertices;
  for (unsigned int k = 0; k < am->mNumAnimMeshes; ++k) {
    const aiAnimMesh *a = am->mAnimMeshes[k];
    if (!a)
      continue;
    model::morph_target mt;
    mt.name = a->mName.C_Str();
    mt.position_deltas.assign(nv, geometry::point_t{{0.0, 0.0, 0.0}});
    if (a->HasPositions() && a->mNumVertices == nv)
      for (unsigned int v = 0; v < nv; ++v)
        mt.position_deltas[v] = {{a->mVertices[v].x - am->mVertices[v].x,
                                  a->mVertices[v].y - am->mVertices[v].y,
                                  a->mVertices[v].z - am->mVertices[v].z}};
    if (a->HasNormals() && am->HasNormals() && a->mNumVertices == nv) {
      mt.normal_deltas.reserve(nv);
      for (unsigned int v = 0; v < nv; ++v)
        mt.normal_deltas.push_back(
            {{a->mNormals[v].x - am->mNormals[v].x, a->mNormals[v].y - am->mNormals[v].y,
              a->mNormals[v].z - am->mNormals[v].z}});
    }
    mm.morphs.push_back(std::move(mt));
    mm.morph_weights.push_back(a->mWeight);
  }
}

// The node a morph channel names: a node of that name, else the first node drawing a mesh of it.
int morph_node(const model &m, const std::string &name) {
  const int n = node_named(m, name);
  if (n >= 0)
    return n;
  for (std::size_t i = 0; i < m.nodes.size(); ++i)
    for (int mi : m.nodes[i].meshes)
      if (mi >= 0 && mi < static_cast<int>(m.meshes.size()) && m.meshes[mi].name == name)
        return static_cast<int>(i);
  return -1;
}

// The scene's animations as clips, their times in seconds.
void read_clips(const aiScene *s, model &m) {
  for (unsigned int a = 0; a < s->mNumAnimations; ++a) {
    const aiAnimation *an = s->mAnimations[a];
    if (!an)
      continue;
    model::clip c;
    c.name = an->mName.C_Str();
    const double tps = an->mTicksPerSecond > 0.0 ? an->mTicksPerSecond : 25.0;
    double last = 0.0;
    for (unsigned int k = 0; k < an->mNumChannels; ++k) {
      const aiNodeAnim *na = an->mChannels[k];
      if (!na)
        continue;
      model::clip::node_channel ch;
      ch.node = node_named(m, na->mNodeName.C_Str());
      if (ch.node < 0)
        continue;
      for (unsigned int i = 0; i < na->mNumPositionKeys; ++i) {
        const aiVectorKey &key = na->mPositionKeys[i];
        ch.position.t.push_back(key.mTime / tps);
        ch.position.v.push_back({{key.mValue.x, key.mValue.y, key.mValue.z}});
        ch.position.step = key.mInterpolation == aiAnimInterpolation_Step;
        last = std::max(last, key.mTime / tps);
      }
      for (unsigned int i = 0; i < na->mNumRotationKeys; ++i) {
        const aiQuatKey &key = na->mRotationKeys[i];
        ch.rotation.t.push_back(key.mTime / tps);
        ch.rotation.q.push_back({{key.mValue.w, key.mValue.x, key.mValue.y, key.mValue.z}});
        ch.rotation.step = key.mInterpolation == aiAnimInterpolation_Step;
        last = std::max(last, key.mTime / tps);
      }
      for (unsigned int i = 0; i < na->mNumScalingKeys; ++i) {
        const aiVectorKey &key = na->mScalingKeys[i];
        ch.scale.t.push_back(key.mTime / tps);
        ch.scale.v.push_back({{key.mValue.x, key.mValue.y, key.mValue.z}});
        ch.scale.step = key.mInterpolation == aiAnimInterpolation_Step;
        last = std::max(last, key.mTime / tps);
      }
      c.nodes.push_back(std::move(ch));
    }
    for (unsigned int k = 0; k < an->mNumMorphMeshChannels; ++k) {
      const aiMeshMorphAnim *mc = an->mMorphMeshChannels[k];
      if (!mc)
        continue;
      model::clip::weights_channel wc;
      wc.node = morph_node(m, mc->mName.C_Str());
      if (wc.node < 0)
        continue;
      for (unsigned int i = 0; i < mc->mNumKeys; ++i) {
        const aiMeshMorphKey &key = mc->mKeys[i];
        std::vector<double> w;
        for (unsigned int j = 0; j < key.mNumValuesAndWeights; ++j) {
          const unsigned int target = key.mValues[j];
          if (target >= w.size())
            w.resize(target + 1, 0.0);
          w[target] = key.mWeights[j];
        }
        wc.t.push_back(key.mTime / tps);
        wc.weights.push_back(std::move(w));
        last = std::max(last, key.mTime / tps);
      }
      c.weights.push_back(std::move(wc));
    }
    c.duration = an->mDuration > 0.0 ? an->mDuration / tps : last;
    m.clips.push_back(std::move(c));
  }
}

// A read scene as a model; `model_dir` resolves its external textures (empty: none resolve).
model model_of_scene(app &ctx, const aiScene *s, const std::string &model_dir) {
  model m;
  m.materials.reserve(s->mNumMaterials);
  for (unsigned int i = 0; i < s->mNumMaterials; ++i)
    m.materials.push_back(build_material(ctx, s, s->mMaterials[i], model_dir));

  m.meshes.reserve(s->mNumMeshes);
  for (unsigned int i = 0; i < s->mNumMeshes; ++i) {
    const aiMesh *am = s->mMeshes[i];
    model::mesh mm;
    mm.geom = build_geometry(am);
    mm.material = static_cast<int>(am->mMaterialIndex);
    mm.name = am->mName.C_Str();
    m.meshes.push_back(mm);
  }

  if (s->mRootNode)
    read_nodes(s, m);
  // Skins name their bones' nodes, so after the nodes; morph targets; the clips.
  for (unsigned int i = 0; i < s->mNumMeshes && i < m.meshes.size(); ++i) {
    read_skin(s->mMeshes[i], m, m.meshes[i]);
    read_morphs(s->mMeshes[i], m.meshes[i]);
  }
  read_clips(s, m);
  return m;
}

// Shared read path for both handlers.
model read_model_assimp(app &ctx, const std::string &path) {
  Assimp::Importer imp;
  keep_file_axes(imp);
  const aiScene *s = imp.ReadFile(path, import_flags());
  if (!s || (s->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !s->mRootNode)
    throw std::runtime_error(std::string("cvc::model assimp read '") + path +
                             "': " + imp.GetErrorString());
  return model_of_scene(ctx, s, dir_of(path));
}

// The same from bytes in memory, the format by `hint` (an extension without its dot).
model read_model_assimp_memory(app &ctx, const void *data, std::size_t size,
                               const std::string &hint, const model_read_options &options) {
  Assimp::Importer imp;
  keep_file_axes(imp);
  const aiScene *s = imp.ReadFileFromMemory(data, size, import_flags(options), hint.c_str());
  if (!s || (s->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !s->mRootNode)
    throw std::runtime_error("cvc::model assimp read of " + std::to_string(size) + " bytes of ." +
                             hint + ": " + imp.GetErrorString());
  return model_of_scene(ctx, s, std::string());
}

// ── Writing ──────────────────────────────────────────────────────────────────────────────────────

// The assimp export format for an extension (leading dot, lower case) that writes to one file,
// or empty.
std::string export_format(const std::string &ext) {
  if (ext == ".glb")
    return "glb2";
  if (ext == ".ply")
    return "plyb";
  if (ext == ".stl")
    return "stlb";
  return std::string();
}

std::string lower_extension(const std::string &extension) {
  std::string ext = extension;
  if (!ext.empty() && ext[0] != '.')
    ext = "." + ext;
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext;
}

// One cvc mesh's points, normals, colours and uvs as an aiMesh whose faces are its triangles
// (`corners` 3) or its lines (2). The aiMesh owns its arrays (the aiScene frees them).
aiMesh *mesh_to_ai(const geometry &g, const std::string &name, unsigned int material,
                   unsigned int corners) {
  auto *am = new aiMesh();
  am->mName = aiString(name);
  am->mMaterialIndex = material;
  am->mPrimitiveTypes = corners == 3 ? aiPrimitiveType_TRIANGLE : aiPrimitiveType_LINE;
  const unsigned int nv = static_cast<unsigned int>(g.points().size());
  am->mNumVertices = nv;
  am->mVertices = new aiVector3D[nv];
  for (unsigned int v = 0; v < nv; ++v)
    am->mVertices[v] =
        aiVector3D(static_cast<ai_real>(g.points()[v][0]), static_cast<ai_real>(g.points()[v][1]),
                   static_cast<ai_real>(g.points()[v][2]));
  if (corners == 3 && g.normals().size() == nv) {
    am->mNormals = new aiVector3D[nv];
    for (unsigned int v = 0; v < nv; ++v)
      am->mNormals[v] = aiVector3D(static_cast<ai_real>(g.normals()[v][0]),
                                   static_cast<ai_real>(g.normals()[v][1]),
                                   static_cast<ai_real>(g.normals()[v][2]));
  }
  if (g.colors().size() == nv) {
    am->mColors[0] = new aiColor4D[nv];
    for (unsigned int v = 0; v < nv; ++v)
      am->mColors[0][v] =
          aiColor4D(static_cast<ai_real>(g.colors()[v][0]), static_cast<ai_real>(g.colors()[v][1]),
                    static_cast<ai_real>(g.colors()[v][2]), 1);
  }
  if (corners == 3 && g.uvs().size() == nv) {
    am->mNumUVComponents[0] = 2;
    am->mTextureCoords[0] = new aiVector3D[nv];
    for (unsigned int v = 0; v < nv; ++v)
      am->mTextureCoords[0][v] =
          aiVector3D(static_cast<ai_real>(g.uvs()[v][0]), static_cast<ai_real>(g.uvs()[v][1]), 0);
  }
  const std::size_t nf = corners == 3 ? g.tris().size() : g.lines().size();
  am->mNumFaces = static_cast<unsigned int>(nf);
  am->mFaces = new aiFace[nf];
  for (std::size_t f = 0; f < nf; ++f) {
    aiFace &face = am->mFaces[f];
    face.mNumIndices = corners;
    face.mIndices = new unsigned int[corners];
    for (unsigned int k = 0; k < corners; ++k)
      face.mIndices[k] = static_cast<unsigned int>(corners == 3 ? g.tris()[f][k] : g.lines()[f][k]);
  }
  return am;
}

// The aiNode for model node `n` and, below it, its children's: its name and transform, and the
// aiMeshes (`parts`, by model mesh) of each mesh it draws. A file's scene graph is a tree, so a
// node two parents list is written under each; a child that is its own ancestor (`on_path`) is
// left out. Indices out of range are skipped.
aiNode *node_to_ai(const model &m, int n, const std::vector<std::vector<unsigned int>> &parts,
                   std::vector<char> &on_path) {
  const model::node &nd = m.nodes[n];
  auto *an = new aiNode(nd.name);
  const model::transform_t &t = nd.transform;
  an->mTransformation = aiMatrix4x4(
      static_cast<ai_real>(t[0]), static_cast<ai_real>(t[1]), static_cast<ai_real>(t[2]),
      static_cast<ai_real>(t[3]), static_cast<ai_real>(t[4]), static_cast<ai_real>(t[5]),
      static_cast<ai_real>(t[6]), static_cast<ai_real>(t[7]), static_cast<ai_real>(t[8]),
      static_cast<ai_real>(t[9]), static_cast<ai_real>(t[10]), static_cast<ai_real>(t[11]),
      static_cast<ai_real>(t[12]), static_cast<ai_real>(t[13]), static_cast<ai_real>(t[14]),
      static_cast<ai_real>(t[15]));
  std::vector<unsigned int> meshes;
  for (int mi : nd.meshes)
    if (mi >= 0 && mi < static_cast<int>(parts.size()))
      meshes.insert(meshes.end(), parts[mi].begin(), parts[mi].end());
  if (!meshes.empty()) {
    an->mNumMeshes = static_cast<unsigned int>(meshes.size());
    an->mMeshes = new unsigned int[meshes.size()];
    std::copy(meshes.begin(), meshes.end(), an->mMeshes);
  }
  on_path[n] = 1;
  std::vector<aiNode *> children;
  for (int c : nd.children)
    if (c >= 0 && c < static_cast<int>(m.nodes.size()) && !on_path[c]) {
      aiNode *child = node_to_ai(m, c, parts, on_path);
      child->mParent = an;
      children.push_back(child);
    }
  on_path[n] = 0;
  if (!children.empty()) {
    an->mNumChildren = static_cast<unsigned int>(children.size());
    an->mChildren = new aiNode *[children.size()];
    std::copy(children.begin(), children.end(), an->mChildren);
  }
  return an;
}

// A cvc model as an aiScene for the exporter: each mesh's triangles one aiMesh and its lines
// another, and a "default" material for a mesh that has none. The model's node tree is written as
// it is; a model without one has every mesh under the root node.
std::unique_ptr<aiScene> scene_of_model(const model &m) {
  auto scene = std::make_unique<aiScene>();
  std::vector<aiMaterial *> mats;
  for (const material &mat : m.materials) {
    auto *am = new aiMaterial();
    const aiString name(mat.name);
    am->AddProperty(&name, AI_MATKEY_NAME);
    const aiColor4D base(mat.base_color[0], mat.base_color[1], mat.base_color[2],
                         mat.base_color[3]);
    am->AddProperty(&base, 1, AI_MATKEY_BASE_COLOR);
    am->AddProperty(&base, 1, AI_MATKEY_COLOR_DIFFUSE);
    mats.push_back(am);
  }
  const unsigned int fallback = static_cast<unsigned int>(mats.size());
  bool need_fallback = false;
  std::vector<aiMesh *> meshes;
  std::vector<std::vector<unsigned int>> parts(m.meshes.size()); // each model mesh's aiMeshes
  for (std::size_t i = 0; i < m.meshes.size(); ++i) {
    const model::mesh &mm = m.meshes[i];
    const bool has = mm.material >= 0 && mm.material < static_cast<int>(m.materials.size());
    const unsigned int mat = has ? static_cast<unsigned int>(mm.material) : fallback;
    need_fallback = need_fallback || !has;
    if (!mm.geom.tris().empty()) {
      parts[i].push_back(static_cast<unsigned int>(meshes.size()));
      meshes.push_back(mesh_to_ai(mm.geom, mm.name, mat, 3));
    }
    if (!mm.geom.lines().empty()) {
      parts[i].push_back(static_cast<unsigned int>(meshes.size()));
      meshes.push_back(mesh_to_ai(mm.geom, mm.name + ".lines", mat, 2));
    }
  }
  if (need_fallback || mats.empty()) {
    auto *am = new aiMaterial();
    const aiString name("default");
    am->AddProperty(&name, AI_MATKEY_NAME);
    mats.push_back(am);
  }
  scene->mNumMaterials = static_cast<unsigned int>(mats.size());
  scene->mMaterials = new aiMaterial *[mats.size()];
  std::copy(mats.begin(), mats.end(), scene->mMaterials);
  scene->mNumMeshes = static_cast<unsigned int>(meshes.size());
  scene->mMeshes = new aiMesh *[meshes.size()];
  std::copy(meshes.begin(), meshes.end(), scene->mMeshes);
  if (!m.nodes.empty()) {
    std::vector<char> on_path(m.nodes.size(), 0);
    scene->mRootNode = node_to_ai(m, 0, parts, on_path);
    return scene;
  }
  scene->mRootNode = new aiNode("root");
  scene->mRootNode->mNumMeshes = scene->mNumMeshes;
  scene->mRootNode->mMeshes = new unsigned int[scene->mNumMeshes];
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
    scene->mRootNode->mMeshes[i] = i;
  return scene;
}

std::string write_model_assimp_memory(const model &m, const std::string &ext) {
  const std::string format = export_format(ext);
  if (format.empty())
    throw unsupported_model_file_type("cvc::model assimp write: cannot write '" + ext + "'");
  if (m.meshes.empty())
    throw std::runtime_error("cvc::model assimp write: the model has no meshes");
  const std::unique_ptr<aiScene> scene = scene_of_model(m);
  Assimp::Exporter ex;
  const aiExportDataBlob *blob = ex.ExportToBlob(scene.get(), format.c_str(), 0u);
  if (!blob)
    throw std::runtime_error("cvc::model assimp write of " + ext + ": " + ex.GetErrorString());
  if (blob->next)
    throw std::runtime_error("cvc::model assimp write of " + ext + ": more than one file");
  return std::string(static_cast<const char *>(blob->data), blob->size);
}

// -------------
// assimp_model_io
// -------------
// A model_file_io handler that returns the full multi-mesh scene.
class assimp_model_io : public model_file_io {
public:
  assimp_model_io() : _id("assimp_model_io : v1.0"), _extensions(assimp_extensions()) {}

  virtual const std::string &id() const { return _id; }
  virtual const extension_list &extensions() const { return _extensions; }

  virtual model read(app &ctx, const std::string &filename) const {
    return read_model_assimp(ctx, filename);
  }

  virtual void write(app &, const model &m, const std::string &filename) const {
    const std::string::size_type dot = filename.find_last_of('.');
    const std::string ext =
        lower_extension(dot == std::string::npos ? std::string() : filename.substr(dot));
    const std::string bytes = write_model_assimp_memory(m, ext);
    std::ofstream out(filename, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!out)
      throw std::runtime_error("cvc::model assimp write: cannot write " + filename);
  }

private:
  std::string _id;
  extension_list _extensions;
};

// -----------------
// assimp_geometry_io
// -----------------
// A geometry_file_io handler that flattens the scene so read_geometry() works on
// the same formats.
class assimp_geometry_io : public geometry_file_io {
public:
  assimp_geometry_io() : _id("assimp_geometry_io : v1.0") {
    model_file_io::extension_list e = assimp_extensions();
    for (model_file_io::extension_list::const_iterator i = e.begin(); i != e.end(); ++i)
      _extensions.push_back(*i);
  }

  virtual const std::string &id() const { return _id; }
  virtual const extension_list &extensions() const { return _extensions; }

  virtual geometry read(app &ctx, const std::string &filename) const {
    return read_model_assimp(ctx, filename).merged();
  }

  virtual void write(app &, const geometry & /*geom*/, const std::string &filename) const {
    throw unsupported_geometry_file_type(
        std::string("cvc::model assimp write: export not supported: ") + filename);
  }

private:
  std::string _id;
  extension_list _extensions;
};

} // namespace
} // namespace cvc
#endif // CVC_ENABLE_ASSIMP

namespace cvc {
bool can_read_model_from_memory(const std::string &extension) {
#ifdef CVC_ENABLE_ASSIMP
  std::string ext = extension;
  if (!ext.empty() && ext[0] != '.')
    ext = "." + ext;
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const model_file_io::extension_list known = assimp_extensions();
  return std::find(known.begin(), known.end(), ext) != known.end();
#else
  (void)extension;
  return false;
#endif
}

model read_model_from_memory(app &ctx, const void *data, std::size_t size,
                             const std::string &extension) {
  return read_model_from_memory(ctx, data, size, extension, model_read_options());
}

model read_model_from_memory(app &ctx, const void *data, std::size_t size,
                             const std::string &extension, const model_read_options &options) {
  if (!can_read_model_from_memory(extension))
    throw unsupported_model_file_type("cvc::read_model_from_memory: no reader for '" + extension +
                                      "' from memory");
#ifdef CVC_ENABLE_ASSIMP
  std::string hint = extension[0] == '.' ? extension.substr(1) : extension;
  std::transform(hint.begin(), hint.end(), hint.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return read_model_assimp_memory(ctx, data, size, hint, options);
#else
  (void)ctx;
  (void)data;
  (void)size;
  (void)options;
  return model();
#endif
}

bool can_write_model_to_memory(const std::string &extension) {
#ifdef CVC_ENABLE_ASSIMP
  std::string ext = extension;
  if (!ext.empty() && ext[0] != '.')
    ext = "." + ext;
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return !export_format(ext).empty();
#else
  (void)extension;
  return false;
#endif
}

std::string write_model_to_memory(app &ctx, const model &m, const std::string &extension) {
  (void)ctx;
  if (!can_write_model_to_memory(extension))
    throw unsupported_model_file_type("cvc::write_model_to_memory: no writer for '" + extension +
                                      "'");
#ifdef CVC_ENABLE_ASSIMP
  return write_model_assimp_memory(m, lower_extension(extension));
#else
  (void)m;
  return std::string();
#endif
}

// Register the built-in model handlers into ctx's own table (app::registerDefaultHandlers()).
// When Assimp is disabled this registers nothing — read_model then raises "no handler" until
// another backend is added.
void register_default_model_handlers(app &ctx) {
#ifdef CVC_ENABLE_ASSIMP
  ctx.model_handlers().insert(model_file_io::ptr(new assimp_model_io()));
#else
  (void)ctx;
#endif
}

// Register the Assimp geometry flattening handler into ctx's geometry table (as off_io does).
// Called from register_default_geometry_handlers(). Nothing without Assimp.
void register_assimp_geometry_io(app &ctx) {
#ifdef CVC_ENABLE_ASSIMP
  ctx.geometry_handlers().insert(geometry_file_io::ptr(new assimp_geometry_io()));
#else
  (void)ctx;
#endif
}
} // namespace cvc
