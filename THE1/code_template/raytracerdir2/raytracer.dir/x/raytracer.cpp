#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "math.h"
#include <cstring>
#define MAX 9999999999
#define EPSILON 0.0000000000001

using namespace parser;
typedef unsigned char RGB[3];

double max(double x,double y){
    return x>y ? x : y;
}
double min(double x,double y){
    return x<y ? x : y;
}
Vec3f cartesianProduct(Vec3f v1,Vec3f v2){
    Vec3f result;
    result.x = v1.x*v2.x;
    result.y = v1.y*v2.y;
    result.z = v1.z*v2.z;
    return result;
}
//mults a vector with a scaler returns a vector
Vec3f multS(Vec3f a,double s)
{
	Vec3f result;
	result.x = a.x*s;
	result.y = a.y*s;
	result.z = a.z*s;
	return result;
}

//adds two vectors returns a vector
Vec3f add(Vec3f a, Vec3f b)
{
	Vec3f result;
	result.x = a.x+b.x;
	result.y = a.y+b.y;
	result.z = a.z+b.z;
	return result;
}

double dot(Vec3f a,Vec3f b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z;
}

Vec3f normalize(Vec3f a)
{
	return multS(a,1.0/sqrt(dot(a,a)));
}
double clip(double n) {
  return max(0,min(n, 255));
}

//c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
//c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
//c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
Vec3f cross(Vec3f v1,Vec3f v2){
    Vec3f result;
    result.x = v1.y*v2.z - v1.z*v2.y;
    result.y = -(v1.x*v2.z - v1.z*v2.x);
    result.z = v1.x*v2.y - v1.y*v2.x;
    return result;
}
Ray generateRay(int i,int j,Camera* cam){
    Ray result;
    Vec3f m,q,s,u;
    double su,sv,left,right,bottom,top;
    
    left = cam->near_plane.x;
    right = cam->near_plane.y;
    bottom = cam->near_plane.z;
    top = cam->near_plane.w;

    su = (i+0.5)*(right-left)/cam->image_width;
    sv = (j+0.5)*(top-bottom)/cam->image_height;
    
    u = cross(cam->up,multS(cam->gaze,-1));

    m = add(cam->position,multS(cam->gaze,cam->near_distance));
    q = add(m,add(multS(u,left),multS(cam->up,top)));
    s = add(q,add(multS(u,su),multS(cam->up,-sv)));

    result.o = cam->position;
    result.d = add(s,multS(cam->position,-1));

    return result;

}

double intersectSphere(Ray& r,Vec3f& center,double radius){
    double A,B,C;
    double delta;
    Vec3f c;
    double t,t1,t2;

    C = (r.o.x-center.x)*(r.o.x-center.x)+(r.o.y-center.y)*(r.o.y-center.y)+(r.o.z-center.z)*(r.o.z-center.z)-radius*radius;
    
    B = 2*r.d.x*(r.o.x-center.x)+2*r.d.y*(r.o.y-center.y)+2*r.d.z*(r.o.z-center.z);

    A = r.d.x*r.d.x+r.d.y*r.d.y+r.d.z*r.d.z;

    delta = B*B-4*A*C;

    if(delta<0) return -1;
    else if(delta == 0){
        t = -B/(2*A);
    }
    else{
        delta = sqrt(delta);
        A = 2*A;
        t1 = (-B + delta)/A;
        t2 = (-B - delta)/A;
        t = t1<t2 ? t1 : t2;
    }
    return t;
}

Vec3f calcNormalSphere(Vec3f& center,Vec3f& p,float radius){
    Vec3f normal;
    normal = normalize(multS(add(p,multS(center,-1)),1.0/radius));
    return normal;
}
Vec3f calcNormalTri( Vec3f* triangle){
    Vec3f normal;
    normal = cross(add(triangle[2],multS(triangle[1],-1)),add(triangle[0],multS(triangle[1],-1)));
    return normalize(normal);
}
double instersectTriangle(Ray r, Vec3f* triangle,Vec3f& tri_normal){
    Vec3f P,vp,vc,minus_aux;
    double t;

    if(dot(r.d,tri_normal) == 0)
        return -1;
    
    t = dot(add(triangle[0],multS(r.o,-1)),tri_normal)/(dot(r.d,tri_normal));
    
    P = add(r.o,multS(r.d,t));

    minus_aux = add(triangle[0],multS(triangle[1],-1));
    vp = cross(add(P,multS(triangle[1],-1)),minus_aux);
    vc = cross(add(triangle[2],multS(triangle[1],-1)),minus_aux);

    if(dot(vp,vc)<= -EPSILON) return -1;

    minus_aux = add(triangle[1],multS(triangle[2],-1));
    vp = cross(add(P,multS(triangle[2],-1)),minus_aux);
    vc = cross(add(triangle[0],multS(triangle[2],-1)),minus_aux);
    
    if(dot(vp,vc) <= -EPSILON) return -1;

    minus_aux = add(triangle[2],multS(triangle[0],-1));
    vp = cross(add(P,multS(triangle[0],-1)),minus_aux);
    vc = cross(add(triangle[1],multS(triangle[0],-1)),minus_aux);

    if(dot(vp,vc)<= -EPSILON) return -1;

    return t;
     
}

Vec3f computeColor(bool& is_primary,Ray r,Scene* scene,int& left_rec,int camera_index){
    int i,j,k,minS,minTri,minM,minMf;
    Vec3f color,L,N,P,sphereCenter,triangle[3],normal,normal_shadow;
    Vec3f intersection_point;
    Vec3f rec_color = {0,0,0};
    Material material;
    double minT = MAX;
    double t;
    color = {0,0,0};

    minS=minTri=minM=minMf = -1;
    if(left_rec == 0 && is_primary==false){
        return color;
    }

    for(i=0;i<scene->spheres.size();i++){
        sphereCenter = scene->vertex_data[scene->spheres[i].center_vertex_id-1];
        t = intersectSphere(r,sphereCenter,scene->spheres[i].radius);
        if(t<minT && t>= EPSILON){
            minS = i;
            minT = t;
        }
    }
    for(j=0;j<scene->triangles.size();j++){
        
        triangle[0] = scene->vertex_data[scene->triangles[j].indices.v0_id-1];
        triangle[1] = scene->vertex_data[scene->triangles[j].indices.v1_id-1];
        triangle[2] = scene->vertex_data[scene->triangles[j].indices.v2_id-1];
        normal = calcNormalTri(triangle);
        t = instersectTriangle(r,triangle,normal);
        if(t<minT && t>= EPSILON){
            minTri = j;
            minT = t;
        }
    }
    
    for(k=0;k<scene->meshes.size();k++){
        //printf("%d\n",scene->meshes[k].faces.size());
        for(int l=0;l<scene->meshes[k].faces.size();l++){
            
            triangle[0] = scene->vertex_data[scene->meshes[k].faces[l].v0_id-1];
            triangle[1] = scene->vertex_data[scene->meshes[k].faces[l].v1_id-1];
            triangle[2] = scene->vertex_data[scene->meshes[k].faces[l].v2_id-1];
            normal = calcNormalTri(triangle);
            t = instersectTriangle(r,triangle,normal);
            //printf("Mesh k:%d l:%d\n",k,l);
            if(t<minT && t>= EPSILON){
                minM = k;
                minMf = l;
                minT = t;
            }
        }
    }
    if(minT == MAX){
        color.x = scene->background_color.x;
        color.y = scene->background_color.y;
        color.z = scene->background_color.z;
        return color;
    
    };
    
    intersection_point = add(r.o,multS(r.d,minT));
    Vec3f diffuse = {0.0,0.0,0.0},specularity = {0.0,0.0,0.0};
    Vec3f h,w_0,w_0w_1;
    Vec3f ambient;
    
    //is_mesh check
    if(minM > -1){
        triangle[0] = scene->vertex_data[scene->meshes[minM].faces[minMf].v0_id-1];
        triangle[1] = scene->vertex_data[scene->meshes[minM].faces[minMf].v1_id-1];
        triangle[2] = scene->vertex_data[scene->meshes[minM].faces[minMf].v2_id-1];

        normal = calcNormalTri(triangle);
        material = scene->materials[scene->meshes[minM].material_id-1];
        
    
    }
    //is triangle check
    else if(minTri> -1){
        triangle[0] = scene->vertex_data[scene->triangles[minTri].indices.v0_id-1];
        triangle[1] = scene->vertex_data[scene->triangles[minTri].indices.v1_id-1];
        triangle[2] = scene->vertex_data[scene->triangles[minTri].indices.v2_id-1];

        normal = calcNormalTri(triangle);
        material = scene->materials[scene->triangles[minTri].material_id-1];
    }
    //then it is sphere
    else if(minS > -1){
        
        normal = calcNormalSphere(scene->vertex_data[scene->spheres[minS].center_vertex_id-1],intersection_point,scene->spheres[minS].radius);
        material = scene->materials[scene->spheres[minS].material_id-1];
    }

    //calc ambient
    ambient = cartesianProduct(material.ambient,scene->ambient_light);
    color = add(color,ambient);

    //calc diffuse and specularity
  
    Vec3f w_i,i_r2;
    Ray shadow_ray;
    double len_w_i_square,len_w_i;
    double cos_theta_d,cos_theta_s_w_phong;
    bool is_in_shadow;
    
    
    for(int i=0;i<scene->point_lights.size();i++){
        
        minS=minTri=minM=minMf = -1;
        is_in_shadow = false;
        w_i = add(scene->point_lights[i].position,multS(intersection_point,-1));
        len_w_i_square = dot(w_i,w_i);
        len_w_i = sqrt(len_w_i_square);
        
        w_i = normalize(w_i);


        shadow_ray.o = add(intersection_point,multS(normal,scene->shadow_ray_epsilon));
        shadow_ray.d = w_i;


        for(int p=0;p<scene->spheres.size() && !is_in_shadow;p++){
            sphereCenter = scene->vertex_data[scene->spheres[p].center_vertex_id-1];
            t = intersectSphere(shadow_ray,sphereCenter,scene->spheres[p].radius);
            if(t<len_w_i && t>= EPSILON ){
                is_in_shadow=true;
            }
        }
        for(j=0;j<scene->triangles.size() && !is_in_shadow;j++){
            
            triangle[0] = scene->vertex_data[scene->triangles[j].indices.v0_id-1];
            triangle[1] = scene->vertex_data[scene->triangles[j].indices.v1_id-1];
            triangle[2] = scene->vertex_data[scene->triangles[j].indices.v2_id-1];
            normal_shadow = calcNormalTri(triangle);
            t = instersectTriangle(shadow_ray,triangle,normal_shadow);
            if(t<len_w_i && t>= EPSILON){
                is_in_shadow=true;
            }
        }
        
        for(k=0;k<scene->meshes.size() && !is_in_shadow;k++){
            //printf("%d\n",scene->meshes[k].faces.size());
            for(int l=0;l<scene->meshes[k].faces.size() && !is_in_shadow;l++){
                triangle[0] = scene->vertex_data[scene->meshes[k].faces[l].v0_id-1];
                triangle[1] = scene->vertex_data[scene->meshes[k].faces[l].v1_id-1];
                triangle[2] = scene->vertex_data[scene->meshes[k].faces[l].v2_id-1];
                normal_shadow = calcNormalTri(triangle);
                t = instersectTriangle(shadow_ray,triangle,normal_shadow);
                //printf("Mesh k:%d l:%d\n",k,l);
                if(t<len_w_i && t>= EPSILON){
                    is_in_shadow=true;
                }
            }
        }

        if(is_in_shadow) continue;



        cos_theta_d =  max(0,dot(w_i,normal));
        if(cos_theta_d == 0) continue;
        i_r2 = multS(scene->point_lights[i].intensity,(1.0/len_w_i_square));
        diffuse = add(diffuse,cartesianProduct(material.diffuse,multS(i_r2,cos_theta_d)));

        if(is_primary == false)
            w_0 = multS(add(r.o,r.d),-1);
        else
            w_0 = add(scene->cameras[camera_index].position,multS(intersection_point,-1));

        w_0 = normalize(w_0);
        w_0w_1 = add(w_i,w_0);
        w_0w_1 = normalize(w_0w_1);
        h=multS(w_0w_1,sqrt(dot(w_0w_1,w_0w_1)));
        h=normalize(h);
        
        cos_theta_s_w_phong = (double) pow(max(0,dot(normal,h)),material.phong_exponent);
        specularity = add(specularity,cartesianProduct(material.specular,multS(i_r2,cos_theta_s_w_phong)));
        
    }
    if(material.is_mirror == true){
        Ray* refl_r = (Ray*) malloc(sizeof(Ray));
        refl_r->d =  normalize(add(multS(w_0,-1),multS(multS(normal,2),dot(normal,w_0))));
        refl_r->o = intersection_point;
        is_primary=false;
        left_rec--;
        //printf("left rec:%d\n",left_rec);
        rec_color = cartesianProduct(material.mirror,computeColor(is_primary,*refl_r,scene,left_rec,camera_index));
        //printf("%.2f,%.2f,%.2f\n",rec_color.x,rec_color.y,rec_color.z);
    }
     color.x = clip(add(add(diffuse,specularity),add(rec_color,color)).x);
     color.y = clip(add(add(diffuse,specularity),add(rec_color,color)).y);
     color.z = clip(add(add(diffuse,specularity),add(rec_color,color)).z);

     //shadow part


     //printf("R:%.2f G:%.2f B:%.2f\n",color.x,color.y,color.z);
     //sleep(0.001);
    
    return color;
}

int main(int argc, char* argv[])
{
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    
   
    int width,height;
  
    
    unsigned char* image;
    
    
    for(int c=0;c<scene.cameras.size();c++){
        width = scene.cameras[c].image_width;
        height = scene.cameras[c].image_height;
        image = new unsigned char[width*height*3];
        int i=0;
        for(int y=0;y<height;++y){
            for(int x=0;x<width;++x){
                Ray myray = generateRay(x,y,&(scene.cameras[c]));
                int left_rec = scene.max_recursion_depth;
                bool is_primary = true;
                Vec3f rayColor = computeColor(is_primary,myray,&scene,left_rec,c);
                image[i++] = (int) (rayColor.x+0.5);
                image[i++] = (int) (rayColor.y+0.5);
                image[i++] = (int) (rayColor.z+0.5);
            }
        }
        char image_name[scene.cameras[c].image_name.size()+1];
        strcpy(image_name,(scene.cameras[c].image_name).c_str());
        write_ppm(image_name, image, width, height); 
    }
    
}
