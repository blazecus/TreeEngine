// input particle structure
struct Particle {
  pos : vec3<f32>,
  vel : vec3<f32>,
};

// output vertex structure
struct Vertex {
  pos : vec3<f32>,
  norm : vec3<f32>,
};

// uniform cloth parameters
struct SimParams {
  // particle specific parameters
  particleWidth : f32,
  particleHeight : f32,
  particleDist : f32,
  particleMass : f32,
  particleScale : f32,

  // spring constraints 
  closeSpringStrength : f32,
  farSpringStrength : f32,
  outSpringStretch : f32,
  inSpringStretch : f32,

  //wind parameters
  wind_strength : f32,

  // sphere size and movement
  sphereRadius : f32,
  sphereX : f32,
  sphereY : f32,
  sphereZ : f32,

  // time uniforms
  deltaT : f32,
  currentT : f32,
  wind_dir : vec3<f32>,
}

// uniform buffer
@group(0) @binding(0) var<uniform> params : SimParams;
// input particle buffer (for first pass)
@group(0) @binding(1) var<storage, read> particlesSrc : array<Particle>;
// output particle buffer (first and second pass)
@group(0) @binding(2) var<storage, read_write> particlesDst : array<Particle>;
// output vertex buffer (only second pass)
@group(1) @binding(0) var<storage, read_write> vertexOut : array<Vertex>;

// this function calculates all the forces applied to a single particle in the cloth, based on gravity, wind, and springs connected to other particles
fn forces(index: u32, current_pos: vec3<f32>)->vec3<f32>{
  let width :i32= i32(params.particleWidth);
  
  // get particle location
  let x = i32(index) % width;
  let y = i32(index) / width;

  var total_force = vec3<f32>();
  // rest dist determines when forces begin to be applied
  let rest_dist = params.particleDist * 0.95f;

  // spring constants
  let k1 = 73.0f / params.particleScale;
  let k2 = 12.5f / params.particleScale;

  // all 8 surrounding
  for (var addx:i32 = -1; addx < 2; addx++){
    for (var addy:i32 = -1; addy < 2; addy++){
      // apply short spring forces
      var diag_dist = 1.0f;
      if(abs(addx) + abs(addy) == 2){
        diag_dist = 1.41421356237f; //sqrt(2)
      }

      // getting adjacent particles
      let indx:i32 = x + addx;
      let indy:i32 = y + addy;
      var new_index: i32 = indx + indy * width;

      //check bounds
      if(indx >= 0 && indx < width && indy >= 0 && indy < i32(params.particleHeight) && (addx != 0 || addy != 0)){
        // find spring force using spring equation
        let diff = current_pos - particlesSrc[new_index].pos;
        if(rest_dist * diag_dist < length(diff)){
          let spring_force = normalize(diff) * (rest_dist * diag_dist - length(diff)) * k1; // spring equation 
          total_force = total_force + spring_force;
        }
      }

      // repeated spring equations to particles that are farther away
      let farx = indx + addx;
      let fary = indy + addy;
      var long_new_index: i32 = farx + fary * width;

      if(farx >= 0 && farx < width && fary >= 0 && fary < i32(params.particleHeight) && addx != 0 && addy != 0){
        let diff = current_pos - particlesSrc[long_new_index].pos;
        if(rest_dist * diag_dist * 2.0f > length(diff)){
          let spring_force = normalize(diff) * (rest_dist * diag_dist * 2.0f - length(diff)) * k2; 
          total_force = total_force + spring_force;
        }
      }
    }
  }

  // apply force from the moving sphere by direction from center
  let sphere_pos = vec3(params.sphereX, params.sphereY, params.sphereZ);
  let sphere_dist = current_pos - sphere_pos;
  if(length(sphere_dist) < params.sphereRadius){
    let sphere_diff = params.sphereRadius - length(sphere_dist);
    total_force += normalize(sphere_dist) * sphere_diff * sphere_diff * 200.0f;
  }

  // gravity
  total_force.y -= 9.8 * params.particleMass; 

  // wind calculation
  total_force += params.wind_dir * 0.0005f * params.particleScale * params.wind_strength;

  // lock top row of particles 
  var multiplier = 1.0f;
  if(y == i32(params.particleHeight - 1.0f)){
    multiplier = 0.0f;
  }

  return total_force * multiplier; 
}

// converts the position in one cloth square (1,2,3,4) to its relative index position
fn triangle_pos_conversion(square_pos: u32) -> u32{
  var diff = 0; //corner case
  //get proper position index
  if(square_pos == 1u || square_pos == 3u){
    diff = i32(params.particleWidth); //down one
  }
  else if(square_pos == 2u || square_pos == 5u){
    diff = 1; // right one
  }
  else if(square_pos == 4u){
    diff = i32(params.particleWidth) + 1; // down and right one
  }
  
  return u32(diff);
}

// first pass - use RK4 to integrate using force function defined above
@compute
@workgroup_size(64)
fn main(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
  // get index of particle
  let total = arrayLength(&particlesSrc);
  let index = global_invocation_id.x;
  if (index >= total) {
    return;
  }

  // retrieve particle information
  var vPos : vec3<f32> = particlesSrc[index].pos;
  var vVel : vec3<f32> = particlesSrc[index].vel;

  //RK4 integration
  let dt = params.deltaT;

  let k0 = dt * vVel;
  let l0 = dt * forces(index, vPos);
  let k1 = dt * (vVel + l0 * 0.5f);
  let l1 = dt * forces(index, vPos + k0 * 0.5f);
  let k2 = dt * (vVel + l1 * 0.5f);
  let l2 = dt * forces(index, vPos + k1 * 0.5f);
  let k3 = dt * (vVel + l2);
  let l3 = dt * forces(index, vPos + k2);
  
  // integration step
  vPos = vPos + (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
  vVel = vVel + (l0 + 2.0f * l1 + 2.0f * l2 + l3) / 6.0f;

  // convert index to position
  let width = i32(params.particleWidth);
  let height = i32(params.particleHeight);
  let iy:i32 = i32(index) / width;
  let ix:i32= i32(index) % width; 
  
  // constraint loop 
  if(iy < height - 1){
    // constraints are applied by looping through neighbors
    for (var addx:i32 = -1; addx < 2; addx++){
      for (var addy:i32 = -1; addy < 2; addy++){
        let indx:i32 = ix + addx;
        let indy:i32 = iy + addy;
        if(indx >= 0 && indx < width && indy >= 0 && indy < height && (addx != 0 || addy != 0)){
          let new_index : i32 = indx + indy * width;
          let diff = vPos - particlesSrc[new_index].pos;
          var diag_dist = 1.0f;
          if(abs(addx) + abs(addy) == 2){
            diag_dist = 1.41421356237f;
          }
          diag_dist *= params.particleDist;

          // if distance is too far or too low, position is fixed
          if(length(diff) < params.inSpringStretch * diag_dist){
            vPos = particlesSrc[new_index].pos + normalize(diff) * diag_dist * params.inSpringStretch;
          }
          else if(length(diff) > params.outSpringStretch * diag_dist){
            vPos = particlesSrc[new_index].pos + normalize(diff) * diag_dist * params.outSpringStretch;
          }
        }
      }
    }
  }

  // write particle output
  particlesDst[index] = Particle(vPos, vVel);
}

// second pass - convert particles into vertices and faces, indexed properly
@compute
@workgroup_size(64)
fn particle_to_vertex(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
  // get index of particle
  let total = arrayLength(&vertexOut);
  let index = global_invocation_id.x;
  if (index >= total) {
    return;
  }

  // there are about 6 vertices in the buffer per particles - so we map each vertex
  // to their respective face
  let cell: u32 = index / 6u;
  let row: u32 = cell / u32(i32(params.particleWidth - 1.0f));
  var cellIdx: u32 = cell + row;

  // retrieve position from particle
  // particle position is found by finding relative position in square, then triangle
  let square_pos: u32 = index % 6u;
  let vIdx = cellIdx + triangle_pos_conversion(square_pos);
  let vpos :vec3<f32> = particlesDst[vIdx].pos;

  // choose normal calculation method - normals_by_average is smooth, whil normals_by_face shows each face more visibly
  //let norm = normals_by_face(square_pos, cellIdx, vpos);
  let norm = normals_by_average(vIdx, vpos);

  // switch dimensions
  let nv :vec3<f32> = vec3(vpos[2], vpos[0], vpos[1]);
  let nn : vec3<f32> = vec3(norm[2], norm[0], norm[1]);
  vertexOut[index] = Vertex(nv / (0.3f * params.particleScale), nn);
}

// method for generating normals by face - will produce the same normal for all 3 vertices of the same face
fn normals_by_face(square_pos : u32, cellIdx : u32, vpos : vec3<f32>) -> vec3<f32>{
  // check for which side of the square we are on - direction is reversed if on other side
  var add_to_lr: u32 = 0u;
  if(square_pos > 2u){
    add_to_lr = 3u;
  }

  // calculate normal from face edges 
  let left: u32 = ((square_pos + 2u) % 3u) + add_to_lr;
  let right: u32 = ((square_pos + 1u) % 3u) + add_to_lr;

  // backtracking to particle idx from triangle position 
  let lcellIdx = cellIdx + triangle_pos_conversion(left);
  let rcellIdx = cellIdx + triangle_pos_conversion(right);
  
  let lvec :vec3<f32> = vpos - particlesDst[lcellIdx].pos;
  let rvec :vec3<f32> = vpos - particlesDst[rcellIdx].pos;
  return normalize(cross(lvec, rvec));
}

// get normal by averaging normals of all adjacent faces
fn normals_by_average(cellIdx: u32, vpos: vec3<f32>) -> vec3<f32>{  
  // get particle location
  let width :i32= i32(params.particleWidth);
  let height :i32= i32(params.particleHeight);

  let x :i32 = i32(cellIdx) % width;
  let y :i32 = i32(cellIdx) / width;

  // get surrounding particle vectors to origin particle
  var up_particle = vec3<f32>();
  var down_particle = vec3<f32>();
  var left_particle = vec3<f32>();
  var right_particle = vec3<f32>();

  if(y > 0){
    up_particle = normalize(vpos - particlesDst[cellIdx - u32(width)].pos);
  }
  if(y < height - 1){
    down_particle = normalize(vpos - particlesDst[cellIdx + u32(width)].pos);
  }
  if(x > 0){
    left_particle = normalize(vpos - particlesDst[cellIdx - 1u].pos);
  }
  if(x < width - 1){
    right_particle = normalize(vpos - particlesDst[cellIdx + 1u].pos);
  }

  // average normals from surrounding existing faces, weighted by the angle of the corresponding "face" 
  var total_norm = vec3<f32>();
  if(y > 0 && x < width - 1){
    total_norm += cross(up_particle, right_particle) * acos(dot(up_particle, right_particle));
  }
  if(y < height - 1 && x < width - 1){
    total_norm += cross(right_particle, down_particle) * acos(dot(right_particle, down_particle));
  }
  if(y < height - 1 && x > 0){
    total_norm += cross(down_particle, left_particle) * acos(dot(down_particle, left_particle));
  }
  if(y > 0 && x > 0){
    total_norm += cross(left_particle, up_particle) * acos(dot(left_particle, up_particle));
  }
  
  return normalize(total_norm);
}
