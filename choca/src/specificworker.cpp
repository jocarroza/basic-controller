/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


  innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");

  timer.start(Period);


  return true;
}

void SpecificWorker::compute()
{
  float vAdv;
  float vRot;
  
  RoboCompDifferentialRobot::TBaseState bstate;
  std::pair<float, float> coord = target.extraerCoord();
//   std::cout << coord.first << coord.second << endl;
  
  differentialrobot_proxy->getBaseState(bstate);
  innermodel->updateTransformValues("base", bstate.x, 0, bstate.z, 0, bstate.alpha, 0);
  
  if (target.isEmpty() == false){
    
    QVec tr = innermodel->transform("base", QVec::vec3(coord.first, 0, coord.second), "world");
    float d = tr.norm2();
    if (d > 50){
//       vAdv = d;
//       if (vAdv > MAX_ADV)
// 	vAdv = MAX_ADV;
      
      vRot = atan2(tr.x(), tr.z());
      if (vRot > MAX_ROT)
	vRot = MAX_ROT;
    
      if (vRot < -MAX_ROT)
	vRot = -MAX_ROT;

      
      vAdv = MAX_ADV * sigmoid(d)  * gaussian(vRot, 0.3, 0.5);
      differentialrobot_proxy->setSpeedBase(vAdv,vRot);
    }
    else{
      differentialrobot_proxy->setSpeedBase(0, 0);
      target.setEmpty();
    }
  }
  
//     differentialrobot_proxy->setSpeedBase(200,0);
//     TLaserData data = laser_proxy->getLaserData();
//     std::sort(data.begin()+20, data.end()-20,[](auto a,auto b){return a.dist<b.dist;});
//     if (data[20].dist < 300){
//       if (rand() % 2 == 0){
// 	giro = -0.8;
//       }
//       else{
// 	giro = 0.8;
//       }
//       
//       differentialrobot_proxy->setSpeedBase(0, giro);
//       int tiempo = rand() % 10 + 1;
//       usleep(tiempo*100000);
//     }
    
  
//   }
//   for(auto d:data)
//     qDebug()<<d.dist<<d.angle;
//   differentialrobot_proxy->setSpeedBase(100,0);
  
//   differentialrobot_proxy->setSpeedBase(0,0);
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

float SpecificWorker::sigmoid(float d)
{	
  return (1.f / (1.f + exp(-d))) - 0.5;
}

float SpecificWorker::gaussian(float vr, float vx, float h)
{
  float landa = -(vx*vx)/log(h);
  return exp(-(vr*vr)/landa);
}

void SpecificWorker::setPick(const Pick& myPick)
{
  //std::cout << myPick.x << myPick.z << endl;
  target.insertarCoord(myPick.x, myPick.z);
}








