#pragma once

#include <glm/glm.hpp>
#include <webgpu/webgpu.hpp>

#include <ResourceManager.h>
#include <array>
#include <vector>

class ClothObject {
public:
  // (Just aliases to make notations lighter)
  using vec3 = glm::vec3;
  using vec2 = glm::vec2;
  using mat3x3 = glm::mat3x3;

  // buffer members
  // two particle buffers that alternate each frame - one input, one output
  std::array<wgpu::Buffer, 2> particleBuffers = {nullptr, nullptr};
  wgpu::Buffer m_vertexBuffer = nullptr;
  wgpu::Buffer m_uniformBuffer = nullptr;
  wgpu::ShaderModule m_shaderModule = nullptr;

  // webgpu data structures
  wgpu::BindGroupLayout m_bindGroupLayouts[2] = {nullptr, nullptr};
  wgpu::BindGroup m_bindGroup = nullptr;
  wgpu::BindGroup m_vertexBindGroup = nullptr;
  wgpu::PipelineLayout m_pipelineLayout = nullptr;
  wgpu::ComputePipeline m_pipeline = nullptr;
  wgpu::PipelineLayout m_vertexPipelineLayout = nullptr;
  wgpu::ComputePipeline m_vertexPipeline = nullptr;

  // buffer size used in initialization
  int m_bufferSize = 0;

  // vertex output structure for compute shader
  struct ClothVertex {
    // garbage is necessary for 32 byte blocks
    vec3 position;
    float garbage1;
    vec3 normal;
    float garbage2;
  };

  // particle input structure for compute shader
  struct ClothParticle {
    // garbage values used to reach 32 byte blocks
    vec3 position;
    float garbage1;
    vec3 velocity;
    float garbage2;
  };

  // fixed cloth parameter structure
  struct ClothParameters {
    int width = 100;
    int height = 100;
    int particlesPerGroup = 64;

    float scale = 1.0f;
    float massScale = 100.0f;
    float maxStretch = 1.1f;
    float minStretch = 0.1f;
    float closeSpringStrength = 73.0f;
    float farSpringStrength = 12.5f;

    vec3 wind_dir = vec3(0.0f, 0.0f, 1.0f);
    float wind_strength = 10.0f;

    float sphereRadius = 0.3f;
    float spherePeriod = 150.0f;
    float sphereRange = 2.0f;
    float deltaT = 0.008f;
  };

  // compute shader uniform data structure
  struct ClothUniforms {
    float width;
    float height;

    float particleDist;
    float particleMass;
    float particleScale;

    float closeSpringStrength;
    float farSpringStrength;
    float maxStretch;
    float minStretch;

    // wind parameters
    float wind_strength;

    float sphereRadius;
    float sphereX;
    float sphereY;
    float sphereZ;

    float deltaT;
    float currentT;
    vec3 wind_dir;
    float garbage1; // garbage for vec3
  };

  // data structure members
  ClothParameters parameters = ClothParameters();
  ClothUniforms uniforms = ClothUniforms();

  // additional parameters, determined from ClothParameters
  int numParticles = parameters.width * parameters.height;
  int numVertices = 3 * 2 * (parameters.width - 1) * (parameters.height - 1);
  float totalMass = parameters.scale * parameters.massScale;
  float particleMass = totalMass / numParticles;
  float particleDist = parameters.scale / parameters.height;

  // time variables
  float currentT = 0.0f;
  int frame = 0;
  vec3 sphere_pos = vec3(0.0f, 0.0f, -1.0f);

  // functions
  void updateParameters(ClothParameters &p);

  void processFrame(wgpu::Device &device);
  void computePass(wgpu::Device &device);

  void updateUniforms(wgpu::Device &device);
  void terminateUniforms();

  void fillBuffer(wgpu::Device &device);
  void initBuffers(wgpu::Device &device);
  void terminateBuffers();

  void initBindGroup(wgpu::Device &device);
  void terminateBindGroups();

  void initBindGroupLayout(wgpu::Device &device);
  void terminateBindGroupLayouts();

  void initComputePipeline(wgpu::Device &device);
  void terminateComputePipeline();

  void initiateNewCloth(ClothParameters &p, wgpu::Device &device);
  void terminateAll();
};
