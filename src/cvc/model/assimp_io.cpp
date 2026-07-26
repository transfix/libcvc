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

#include <cvc/model/model_file_io.h>

#ifdef CVC_ENABLE_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/texture.h>
#include <cstdio>
#include <cstdlib>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/image/image.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

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

// Decode an embedded, compressed texture blob (mHeight == 0: pcData is `mWidth`
// bytes of e.g. PNG/JPEG). cvc's image reader only reads from a path, so spill
// the blob to a temp file with the format-hint extension and read_image() it.
image decode_embedded_compressed(const aiTexture *tex) {
  std::string hint(tex->achFormatHint);
  std::string ext = hint.empty() ? std::string("bin") : hint;
  static int counter = 0;
  std::string tmp = default_temp_dir() + "/cvc_embedded_tex_" +
                    std::to_string(static_cast<long long>(counter++)) + "." + ext;
  {
    std::ofstream ofs(tmp.c_str(), std::ios::binary);
    if (!ofs)
      throw std::runtime_error("could not open temp file for embedded texture");
    ofs.write(reinterpret_cast<const char *>(tex->pcData),
              static_cast<std::streamsize>(tex->mWidth));
  }
  image out;
  try {
    out = read_image(tmp);
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
// texture is non-fatal: return an empty image (the path is still recorded).
image load_texture(const aiScene *scene, const std::string &texpath, const std::string &model_dir) {
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
      return tex->mHeight == 0 ? decode_embedded_compressed(tex) : decode_embedded_raw(tex);
    }
    std::string resolved = is_absolute_path(texpath) ? texpath : (model_dir + "/" + texpath);
    return read_image(resolved);
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
      // Post-Triangulate every polygon is a triangle; skip anything else
      // (points/lines emitted by degenerate faces) defensively.
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
material build_material(const aiScene *scene, const aiMaterial *aim, const std::string &model_dir) {
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
    mat.base_color_texture = load_texture(scene, mat.base_color_texture_path, model_dir);
  }

  return mat;
}

// Shared read path for both handlers.
model read_model_assimp(const std::string &path) {
  Assimp::Importer imp;
  const aiScene *s = imp.ReadFile(path, import_flags());
  if (!s || (s->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !s->mRootNode)
    throw std::runtime_error(std::string("cvc::model assimp read '") + path +
                             "': " + imp.GetErrorString());

  const std::string model_dir = dir_of(path);

  model m;
  m.materials.reserve(s->mNumMaterials);
  for (unsigned int i = 0; i < s->mNumMaterials; ++i)
    m.materials.push_back(build_material(s, s->mMaterials[i], model_dir));

  m.meshes.reserve(s->mNumMeshes);
  for (unsigned int i = 0; i < s->mNumMeshes; ++i) {
    const aiMesh *am = s->mMeshes[i];
    model::mesh mm;
    mm.geom = build_geometry(am);
    mm.material = static_cast<int>(am->mMaterialIndex);
    mm.name = am->mName.C_Str();
    m.meshes.push_back(mm);
  }

  return m;
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

  virtual model read(const std::string &filename) const { return read_model_assimp(filename); }

  virtual void write(const model & /*m*/, const std::string &filename) const {
    throw unsupported_model_file_type(
        std::string("cvc::model assimp write: export not supported: ") + filename);
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

  virtual geometry read(const std::string &filename) const {
    return read_model_assimp(filename).merged();
  }

  virtual void write(const geometry & /*geom*/, const std::string &filename) const {
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
// Register the built-in model handlers. Called once (lazily) from
// model_file_io::get_handlers(). When Assimp is disabled this registers nothing
// — read_model then raises "no handler" until another backend is added.
void register_default_model_handlers() {
#ifdef CVC_ENABLE_ASSIMP
  model_file_io::insert_handler(model_file_io::ptr(new assimp_model_io()));
#endif
}

#ifdef CVC_ENABLE_ASSIMP
// Register the Assimp geometry flattening handler into the geometry_file_io
// registry (same mechanism as off_io). Called from
// register_default_geometry_handlers().
void register_assimp_geometry_io() {
  geometry_file_io::insert_handler(geometry_file_io::ptr(new assimp_geometry_io()));
}
#endif
} // namespace cvc
