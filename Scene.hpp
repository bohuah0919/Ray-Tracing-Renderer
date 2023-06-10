#pragma once
#include"Object.hpp"
#include"BVH.hpp"
class Scene {
public:
	int width, height;
    float fov = 40.0f;
	Scene(int w, int h): width(w), height(h){}
	void addObj(Object* obj) {objList.push_back(obj);}
	void buildBVH() { 
		sceneBVH = new BVH(objList);
	}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (sceneBVH) {
			inter = sceneBVH->getIntersection(ori, dir);
		}
		return inter;
	}
    void sampleLight(Intersection& inter, float& pdf) {
        float totalArea = 0;
        float area_sum = 0;
        float p = getRandomFloat();
        for (auto obj : objList) {
            if (obj->isLight()) totalArea += obj->getArea();
        }
        for (int i = 0; i < objList.size(); i++) {
            if (objList[i]->isLight()) {
                area_sum += objList[i]->getArea();
                if (area_sum >= p * totalArea) {
                    objList[i]->sample(inter, pdf);
                    break; 
                }
            }
        }
    }
    Eigen::Vector3f castRay(Eigen::Vector3f ori, Eigen::Vector3f dir)
    {
        Intersection p_inter = getIntersection(ori, dir);
        if (!p_inter.hitHappened) return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        if (p_inter.obj->isLight())
            return p_inter.material->emit;

        Eigen::Vector3f p = p_inter.pos;
        Eigen::Vector3f N = p_inter.normal;
        Eigen::Vector3f wo = -dir;

        Intersection inter_light;
        float pdf_light;
        sampleLight(inter_light, pdf_light);
        Eigen::Vector3f x = inter_light.pos;
        Eigen::Vector3f ws = (x - p).normalized();
        Eigen::Vector3f light_normal = inter_light.normal;
        Eigen::Vector3f emit = inter_light.material->emit;
        Eigen::Vector3f L_dir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

        Intersection block_inter = getIntersection(p + 0.00001f * N, ws);
        if (p_inter.material->mType == SPECULAR) L_dir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        else if (fabs(block_inter.distance - (x - p).norm())<0.001f)
            L_dir = emit.cwiseProduct(p_inter.material->eval(wo, ws, N)) * std::max(ws.dot(N),0.0f) * std::max(-ws.dot(light_normal),0.0f) / ((x - p).norm() * (x - p).norm()) / pdf_light;
        
        Eigen::Vector3f L_indir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        float prob = getRandomFloat();
        if (prob > 0.8f) return L_dir + L_indir;
        Eigen::Vector3f wi = p_inter.material->sampleDir(wo, N);

        Intersection obj_inter = getIntersection(p + 0.00001f * N, wi);
        if (obj_inter.hitHappened && p_inter.material->mType == SPECULAR)
            return castRay(p + 0.00001f * N, wi).cwiseProduct(p_inter.material->eval(wo, wi, N)) / p_inter.material->PDF(wo, wi, N) / 0.8f;
        if (obj_inter.hitHappened && !obj_inter.obj->isLight())
            L_indir = castRay(p + 0.00001f * N, wi).cwiseProduct(p_inter.material->eval(wo, wi, N)) * std::max(wi.dot(N),0.0f) / p_inter.material->PDF(wo, wi, N) / 0.8f;  

        Eigen::Vector3f color =  L_dir + L_indir;
        color(0) = std::max(0.0f, std::min(1.0f, color.x()));
        color(1) = std::max(0.0f, std::min(1.0f, color.y()));
        color(2) = std::max(0.0f, std::min(1.0f, color.z()));
        return color;

    }
	
private:
	std::vector<Object*> objList;
	BVH* sceneBVH;
};