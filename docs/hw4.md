# HW4: Textures & Perlin Noise
<video controls autoplay muted loop width="100%">
  <source src="../assets/videos/tunnel_of_doom.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Hello everyone, welcome to the part 4 of our ray-tracing journey. Today’s topic is textures. Previously my ray-tracer only takes the colors of the objects from the light sources. But in real life most of the objects have textures on them. So by adding textures features to my ray-tracer I will obtain some visually cool scenes. Also in this blog we will not only mention about the texture mapping but also see normal and bump mappings. In addition, we will generate some artistic textures with the help of the Perlin Noise.

## 1. Texture Mapping
In this part we will talk about the background, sphere and triangle textures. Also at the and we will discuss the 2 interpolation methods which are nearest and bilinear. Also we have some different modes for rendering. Here is the list:

* replace_kd: Replace diffuse shading component with the texture color
* replace_ks: Replace specular shading component with the texture color
* replace_all: Replace the pixel color with the texture color
* blend_kd: Mix diffuse shading component with the texture color and the original kd component

### 1.1 Interpolation Modes

We have two interpolation modes for our ray-tracer which are nearest and bilinear interpolation. When we get the texture coordinates it is a low chance that this coordinate directly the pixel coordinate of the texture. Therefore, we need to find the color value of that coordinates.

* Nearest: Get the closest pixel’s color value
* Bilinear: Take the distance average of the closest 4 pixel

Nearest Interpolation Code:
```cpp
int w, h;
int ceil_w = ceil(tex_w);
int floor_w = floor(tex_w);
int ceil_h = ceil(tex_h);
int floor_h = floor(tex_h);

if (ceil_w - tex_w < tex_w - floor_w) w = ceil_w;
else w = floor_w;
if (ceil_h - tex_h < tex_h - floor_h) h = ceil_h;
else h = floor_h;

color = fetchImage(image, width, height, offset, w, h);
```

Bilinear Interpolation Code:

```cpp
int floor_w = floor(tex_w);
int floor_h = floor(tex_h);

real dx = tex_w - floor_w;
real dy = tex_h - floor_h;

color = fetchImage(image, width, height, offset, floor_w, floor_h) * (1-dx) * (1-dy) +
        fetchImage(image, width, height, offset, floor_w + 1, floor_h) * (dx) * (1-dy) +
        fetchImage(image, width, height, offset, floor_w, floor_h + 1) * (1-dx) * (dy) +
        fetchImage(image, width, height, offset, floor_w + 1, floor_h + 1) * (dx) * (dy);

```
Nearest Interpolation Output: 
![plane_nearest](assets/images/hw4/plane_nearest.png)

Bilinear Interpolation Output: 
![plane_bilinear](assets/images/hw4/plane_bilinear.png)

As one can observe that aliasing effect significantly reduced when bilinear interpolation is applied. But there is still noticeable aliasing and that can be solved with using trilinear interpolation. I did not implement it for this homework because of the restricted time that I have. But in future I want to implement it and add the results here.

### 1.2 Background Textures
This texture is like the image plane’s texture. We will apply background texture only if our ray did not hit any object.

Here is the of this:
```cpp
Vec3r color(0.0);
    
    real tex_w, tex_h;
    tex_w = (real)(curr_w * texImg.width) / img_width;
    tex_h = (real)(curr_h * texImg.height) / img_height;

    if (texMap.interpolation == "bilinear") {
        color = applyBilinearInterpolation(texImg.data, texImg.width, texImg.height, texImg.channels, tex_w, tex_h);
    }
    if (texMap.interpolation == "nearest") {
        color = applyNearestInterpolation(texImg.data, texImg.width, texImg.height, texImg.channels, tex_w, tex_h);
    }
    return color * 255;
```
The texture coordinates computed from the image plane’s width and height. And the corresponding texture color assigned as the color of the current pixel.

![galactica_static_only_background](assets/images/hw4/galactica_static_only_background.png)

### 1.3 Sphere Textures
For texture mapping on spheres we need to map the surface points on the sphere to a texture coordinates.

Here is the visualization of this mapping:
![sphere_mapping](assets/images/hw4/sphere_mapping.webp)

As one can observe that theta and phi can be written in terms of x,y,z. To find the texture coordinates we need to map these angles to the u,v (the final equation given in the image).

Here is the corresponding code:
![sphere_nearest_bilinear](assets/images/hw4/sphere_nearest_bilinear.png)

### 1.4 Triangle Textures
For triangles texture mapping we can find the texture coordinates by using the barycentric coordinates of the triangle.

![triangle_texture_formula](assets/images/hw4/triangle_texture_formula.webp)

Here is the corresponding code:
```cpp
real tex_u = texCoord_v0.x + beta * (texCoord_v1.x - texCoord_v0.x) + gamma * (texCoord_v2.x - texCoord_v0.x);
real tex_v = texCoord_v0.y + beta * (texCoord_v1.y - texCoord_v0.y) + gamma * (texCoord_v2.y - texCoord_v0.y);

tex_u -= floor(tex_u);
tex_v -= floor(tex_v);
```

One needs to careful about the last two code pieces. These are necessary for tiling. If we do not do this then our texture will be stretched across the triangle.

Here are the results:
![wood_box_no_specular_original](assets/images/hw4/wood_box_no_specular_original.png)

![wood_box_original](assets/images/hw4/wood_box_original.png)

![galactica_static_no_bump](assets/images/hw4/galactica_static_no_bump.png)

## 2. Normal Mapping
In normal mapping there is no geometry modification but we slightly perturbed the normals by using a normal map textures. Normal maps are defined in a canonical tangent space of the surface. We need to rotate the canonical tangent space (U, V, W) to the real tangent space (T,B,N). T is the tangent vector, B is the bitangent vector and N is the surface normal. T, B, N corresponds to U, V, W in texture space.

So now we need to compute the tangent vectors of the sphere and triangle.

Derivation of the sphere’s tangent vectors:
![normal_mapping_formula](assets/images/hw4/normal_mapping_formula.webp)

Our aim is to find the change of x,y,z coordinates with respect to u,v

In the sphere case we can compute the surface normal by this:
* **N = B x T**
  
Derivation of the triangle’s tangent vectors:
![triangle_tangent](assets/images/hw4/triangle_tangent.webp)

We already compute the parameters on the right hand side so we can simply multiply these matrices to find the tangent and bitangent vectors

Next we can compute the new normal by the transformation matrix that will rotate UVW to TBN:
Derivation of the triangle’s tangent vectors:
![uvw_to_tbn](assets/images/hw4/uvw_to_tbn.webp)

Here is the corresponding code:

```cpp
Vec3r Sphere::computeTangentU(Vec3r local_p, real pi) {
    Vec3r tangent_u;
    tangent_u.x = 2 * pi * local_p.z;
    tangent_u.y = 0;
    tangent_u.z = 2 * pi * local_p.x;
    return normalizeVec3r(tangent_u);
}


Vec3r Sphere::computeTangentV(Vec3r local_p, real pi, real phi, real theta) {
    Vec3r tangent_v;
    tangent_v.x = -pi * local_p.y * cos(phi);
    tangent_v.y = -pi * radius * sin(theta);
    tangent_v.z = -pi * local_p.y * sin(phi);
    return normalizeVec3r(tangent_v);
}

Vec3r Sphere::computeNormalMapNormals(Vec3r T, Vec3r B, Vec3r N, Vec3r MN) {
    Vec3r new_normal;
    new_normal.x = T.x * MN.x + B.x * MN.y + N.x * MN.z;
    new_normal.y = T.y * MN.x + B.y * MN.y + N.y * MN.z;
    new_normal.z = T.z * MN.x + B.z * MN.y + N.z * MN.z;
    return normalizeVec3r(new_normal);
}
```

cube_wall and cube_wall_normal scenes:
![cube_wall](assets/images/hw4/cube_wall.png)

![cube_wall_normal](assets/images/hw4/cube_wall_normal.png)

![brickwall_with_normalmap](assets/images/hw4/brickwall_with_normalmap.png)

![sphere_normal_orginal](assets/images/hw4/sphere_normal_orginal.png)

![cube_waves](assets/images/hw4/cube_waves.png)

![cube_cushion](assets/images/hw4/cube_cushion.png)

## 3. Bump Mapping

In bump mapping we perturbed the surface by a height or displacement function (d) along the surface normal. Again the geometry is not changed but the surface normals are.

![bump_mapping_illustration](assets/images/hw4/bump_mapping_illustration.webp)

Bump Mapping Derivation:
* The original surface is defined as a function of two parameter: p(u,v)
* We tangent and bitangent can be computed as we proposed
* The bump surface is: q(u,v) = p(u,v) + h(u,v) * n(u,v)
* Then n_new = bitangent x tangent
  
![small_term](assets/images/hw4/small_term.webp)

* We already now the tangent and bitangent vectors
* The partial derivative of the height function can be computed numerically

![bump_mapping_derivation](assets/images/hw4/bump_mapping_derivation.webp)
where H and W are texture dimensions

Here is the corresponding code:

```cpp
Vec3r c = sphere.computeReplaceComponent(tex_map, tex_img, tex_u, tex_v);
real h = (c.x + c.y + c.z) / 3;

real epsilon_u = 1.0 / tex_img.width;
Vec3r c_u = sphere.computeReplaceComponent(tex_map, tex_img, tex_u + epsilon_u, tex_v);
real h_u = (c_u.x + c_u.y + c_u.z) / 3;

real epsilon_v = 1.0 / tex_img.height;
Vec3r c_v = sphere.computeReplaceComponent(tex_map, tex_img, tex_u, tex_v + epsilon_v);
real h_v = (c_v.x + c_v.y + c_v.z) / 3;

real dh_du = (h_u - h) * tex_map.bump_factor / epsilon_u;
real dh_dv = (h_v - h) * tex_map.bump_factor / epsilon_v;

Vec3r local_normal = normalizeVec3r(local_p);
Vec3r dq_du = tangent_u + dh_du * local_normal;
Vec3r dq_dv = tangent_v + dh_dv * local_normal;
Vec3r new_normal = normalizeVec3r(crossVec3r(dq_du, dq_dv));

Mat4r invTr = sphere.getInverseTransformation().transpose();
hit_record.normal = applyTransformToVector(invTr, new_normal);
```

Also partial derivatives of the height function is multiplied by a bump factor in order to control the effect of bumping.

sphere_nobump_justbump and sphere_nobump_bump scenes:

![sphere_nobump_justbump](assets/images/hw4/sphere_nobump_justbump.png)

![sphere_nobump_bump](assets/images/hw4/sphere_nobump_bump.png)

My bump mapping on spheres are problematic as one can observe from the second image. I debugged and decided that it is because of my maybe wrong tangent vectors computation. Because they are not perpendicular to each other. However in normal mapping this will not affect the results but in future I will correct it and share the results with you.

killeroo_bump_wall scene with tiling and without tiling:

![killeroo_bump_walls_wo_tiling](assets/images/hw4/killeroo_bump_walls_wo_tiling.png)

![killeroo_bump_walls](assets/images/hw4/killeroo_bump_walls.png)

## 4. Perlin Noise
Adding textures to the objects from a different image source is not the only option. We can generate the textures on-the-fly also. In this part we will talk about the Perlin Noise (founded by Ken Perlin) and how one can generate procedural textures using this method.

Perlin Noise Generation Algorithm:

* Associate some random gradient vectors at lattice corners (8 in 3D)
* Compute 8 vectors from the corners
* Compute the dot product of each vector with the corresponding gradient vector
  
![perlin_noise_cell](assets/images/hw4/perlin_noise_cell.webp)

* Compute the contribution of each corner based on the distance of the point to the corners

![perlin_function](assets/images/hw4/perlin_function.webp)

![perlin_mult](assets/images/hw4/perlin_mult.webp)

* Add up everything

![perlin_add](assets/images/hw4/perlin_add.webp)

Perlin proposes this gradient vectors for 3D noise:
![perlin_gradients](assets/images/hw4/perlin_gradients.png)

Here is the corresponding code:

```cpp
real Perlin::noise(Vec3r point, std::string conversion) {
    int i = floor(point.x);
    int j = floor(point.y);
    int k = floor(point.z);

    real c = 0.0;

    for (int a = 0; a < 8; ++a) {
        Vec3i ijk = {i, j, k};
        ijk = ijk + addTable[a];

        int idx;
        idx = table[abs(ijk.z) % 16];
        idx = table[abs(ijk.y + idx) % 16];
        idx = table[abs(ijk.x + idx) % 16];
        Vec3r g = gradients[idx];

        real dx = point.x - ijk.x;
        real dy = point.y - ijk.y;
        real dz = point.z - ijk.z;
        Vec3r d = {dx, dy, dz};

        c += f(dx) * f(dy) * f(dz) * dotVec3r(g, d);
    }

    if (conversion == "absval") {
        c = abs(c);
    }
    else {
        c = (c + 1) / 2;
    }
    if (c < 0.001) c = 0.0;

    return c;
}

real Perlin::f(real x) {
    real result = 0;
    if (abs(x) < 1) {
        result = -6 * pow(abs(x), 5) + 
                 15 * pow(abs(x), 4) +
                -10 * pow(abs(x), 3) + 1;
    }
    return result;
}
```

As one can observe that gradient vectors are shuffled by accessing them through a table that contains the indices

![sphere_perlin](assets/images/hw4/sphere_perlin.png)

![sphere_perlin_scale](assets/images/hw4/sphere_perlin_scale.png)

Perlin noise can be scaled by a constant to control the frequency of the noise

![sphere_perlin_bump](assets/images/hw4/sphere_perlin_bump.png)

![cube_perlin](assets/images/hw4/cube_perlin.png)

![cube_perlin_bump](assets/images/hw4/cube_perlin_bump.png)

Also we can define how many octaves that noise function should contain. An octave is defined as the noise function evaluated at twice the frequency but weighted with half the weight. Here is the code snippet

```cpp
for (int i = 1; i < tex_map.num_octaves; ++i) {
    noise += pow(2, -i) * per.noise(local_p * pow(2,i),  tex_map.noise_conversion);
}
```

![dragon_new](assets/images/hw4/dragon_new.png)

## 5. Summary

Overall dealing with the textures is a great experience for me. We obtained some cool scenes and also increase the reality of our scenes. I have some bugs such as sphere’s bump mapping but I will try to solve them in future. Thanks for reading my blog! See you guys in my next blog which will be about lighting.

Here are the other scenes that I have not mentioned above

![ellipsoids_texture](assets/images/hw4/ellipsoids_texture.png)

![veachajar.png](assets/images/hw4/veachajar.png.webp)

VeachAjar scene (door is darker due to some lighting effect but the texture is correct). Also the ground is obtained by a checkerboard texture on-the-fly

Checkerboard texture code:

```cpp
Vec3r localP = temp.intersection_point;
real scale = tex_map.scale;
real offset = tex_map.offset;

bool x = ((int) floor((localP.x + offset) * scale)) % 2;
bool y = ((int) floor((localP.y + offset) * scale)) % 2;
bool z = ((int) floor((localP.z + offset) * scale)) % 2;

bool xorXY = x != y;

if (xorXY != z) {
    hit_record.kd = tex_map.black_color;
}
else {
    hit_record.kd = tex_map.white_color;
}
```

![mytap_final.png](assets/images/hw4/mytap_final.png.webp)
