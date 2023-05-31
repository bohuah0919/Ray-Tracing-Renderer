#pragma once
#include"Object.hpp"
#include"BVH.hpp"
class Scene {
public:
	int width, height;
    float fov = 40.0f;
	Scene(float w, float h): width(w), height(h){}
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
        float a = 0;
        float p = getRandomNum();
        for (auto obj : objList) {
            if (obj->isLight()) totalArea += obj->getArea();
        }
        for (int i = 0; i < objList.size(); i++) {
            if (objList[i]->isLight()) {
                a += objList[i]->getArea();
                if (a > p * totalArea) objList[i]->sample(inter, pdf);
            }
        }
    }
    Eigen::Vector3f castRay(Eigen::Vector3f ori, Eigen::Vector3f dir)
    {
        Intersection p_inter = getIntersection(ori, dir);
        if (!p_inter.hitHappened) return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f p = p_inter.pos;
        Eigen::Vector3f N = p_inter.normal;
        Eigen::Vector3f wo = dir;

        Intersection inter_light;
        float pdf_light;
        sampleLight(inter_light, pdf_light);
        Eigen::Vector3f x = inter_light.pos;
        Eigen::Vector3f ws = (x - p).normalized();
        Eigen::Vector3f light_normal = inter_light.normal;
        Eigen::Vector3f emit = inter_light.material->emit;
        Eigen::Vector3f L_dir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

        Intersection block_inter = getIntersection(p, ws);
        if (block_inter.hitHappened && block_inter.obj->isLight())
            L_dir = emit.cwiseProduct(p_inter.material->eval(wo, ws, N)) * std::max(ws.dot(N),0.0f) * abs(- ws.dot(light_normal)) / ((x - p).norm() * (x - p).norm()) / pdf_light;
        if (p_inter.obj->isLight())
            L_dir = emit;


        Eigen::Vector3f L_indir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        float prob = getRandomNum();
        if (prob > 0.8f) return L_dir + L_indir;
        Eigen::Vector3f wi = p_inter.material->sampleDir(N);

        Intersection obj_inter = getIntersection(p, wi);
        if (obj_inter.hitHappened && !obj_inter.obj->isLight())
            L_indir = castRay(p, wi).cwiseProduct(p_inter.material->eval(wo, wi, N)) * std::max(wi.dot(N),0.0f) / p_inter.material->uniformPDF(wo, wi, N) / 0.8f;

        return L_dir + L_indir;

    }
	
private:
	std::vector<Object*> objList;
	BVH* sceneBVH;
};