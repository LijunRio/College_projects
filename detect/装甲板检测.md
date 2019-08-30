### 装甲板检测

----

由于在比赛过程中操作手是第一视角，很难用手动瞄准。通过装甲板检测就是自瞄系统，己方车辆可自动瞄准敌方车辆装甲板，对敌方造成有效的伤害。大大提高了射击精准度。

**功能**：检测装甲板的位置并把位置发送给电控

- 在目标旋转时保证目标的位置的稳定性
- 系统延时尽可能短
- 在多目标时，保证打击的一致性

![](assets/车.png)



#### 整体架构

-----

**文件描述**

| 文件名                 | 作用                                            |
| ---------------------- | ----------------------------------------------- |
| main.cpp               | 算法主函数，包括多线程处理步骤                  |
| caffe_model.cpp        | 自己训练的装甲板贴纸分类模型                    |
| cctag_detector.cpp     | 老版圆形贴纸识别圆形的方法                      |
| coordinate_process.cpp | 根据通信协议对最终左边进行处理的方法            |
| detector_methods.cpp   | 装甲板检测的主要方法（包括预处理，逻辑判断等）  |
| serialport.cpp         | Linux下串口使用的文件                           |
| v4l2.cpp               | Linux下利用V4L2来调节摄像头的曝光饱和度等参数。 |



**装甲板检测流程图**

<img src= assets/绘图1.png width="550">







**基本原理**

1. 如果上一帧的ROI标志位为true 则当前帧的检测区域在上一帧目标的附近，也就是用到了ROl方法，如果为false则全图搜索，此时将与ROI有关的变量全部清零。 
2. 对检测区域内的图进行二值化，也就是预处理，思路就是先将图片用灰度阈值进行二值化，这样子可以将图片中发光的物体给提取出来（装甲板灯条以及日光灯等等），然后再用某种方法将图片中红色或蓝色的区域提取出来，之后再膨胀腐蚀用形态学的方法连通断开的区域，使灯条的形状更加清晰以便于之后的逻辑判断。
   - 一种方法是用RGB的红蓝通道相减，根据设定的阈值得到一张二值图，这种方法虽好，但是在识别蓝色的时候，有时候无法排除掉日光灯干扰，该方法，操作简洁，耗时低。 
   - 另一种是先将图片转化成HSV颜色空间再用通道范围将红色蓝色提取出来，这种方法可以排除掉很多干扰，但是近距离的时候装甲板灯条发白，如果膨胀不到位的话会出现灯条断裂的 情况，膨胀的卷积核过大又会造成预处理耗时过久，因此要权衡一下。 
   - 还有一种是在LAB下利用a通道-b通道来提取红色。这种方法对红色灯条提取的效果是最好的。但是却极容易把比较暗的，有一点带暗红色的物体都提取出来，容易形成过多的早点。

3．在当前二值图内找到所有的轮廓点，用最小旋转矩形将他们包围，此时得到一个个单独的旋转矩形，然后对旋转矩形的四个顶点重新排序，排除长大于宽的噪点，然后根据装甲板灯条的几何特征首先筛除掉一些旋转矩形。

```c++
 if((width/height)>2&&abs(angle)>25&&abs(angle)<70&&){continue;}
```

4．将这些灯条两两再次组成一个大的旋转矩形（也就是候选装甲板），根据一些限制条件筛除掉不符合条件的装甲板，将剩下的待选装甲板放入一个向量中。

```c++
//两两灯条间的角度差不符合的情况
if((abs(angle_1)>5&&abs(angle_1)<20)&&(abs(angle_2)>65&&abs(angle_2)<85)&&((abs(angle_2)>5&&abs(angle_2)<20)&&(abs(angle_1)>65&&abs(angle_1)<85))){
continue;}
```

```c++
//中心距根据灯条的长短分开设置
if(height_1<20||height_2<20){
    cha=10;}
else{
    cha=20;}
//进一步根据两条灯条的中心距和灯条面积比例等条件进一步筛选
//一下条件参数是根据640,480分辨率下设定的，不同分辨率条件值可能不一样
if(lantern_cha<cha&&(area_rate>rate2&&area_rate<rate1)) 
{
	double diameter=sqrt(pow(rectPoint[original_index_i].center.x-			rectPoint[original_index_j].center.x,2)+pow(rectPoint[original_index_i].center.y-rectPoint[original_index_j].center.y,2));
    if(diameter<35){continue;}
    //cout<<"d: "<<diameter<<endl;

    double average_height=(height_1+height_2)*0.5;
    cout<<"h: "<<average_height<<endl;
    double  bili=average_height/diameter;
    if(bili>0.8&&bili<1.8){continue;}
    if((diameter/average_height>2.5&&diameter/average_height<6)||(diameter/average_height>0.3&&diameter/average_height<2.5)){
        Point armour_center=(rectPoint[original_index_i].center+rectPoint[original_index_j].center)*0.5;
        circle(imgOriginal,armour_center,diameter/2,Scalar(0,255,255),1);//紫色
        Vec3f armour_message(armour_center.x,armour_center.y,diameter);
        armour.push_back(armour_message);
    }
}
```

5. 经过上述操作后，我们最终把筛选出来的装甲板存入amour向量中。

   - 如果向量中没有元素，则说明没有找到目标，只有一个的话则这就是最终选择的装甲板。（只有一个装甲板的情况下不再进一步判断装甲板的真实性，及不再传入用装甲板训练的模型。这样做能保证目标的稳定性和代码的效率）
   - 两个以上装夹板的话就得进行接下去的比较，首先我们把距离图像下半部分及比较近的装甲板存为一组，然后从最近的开始用caffe模型进行验证。如果模型给出这个为真，则直接输出。

   ```c++
   else{
       //近的存一组
       vector<Vec3f> near_armour;
       for(int i=0;i<(int)armour.size();i++){
           if(armour[i][1]>imgOriginal.rows/2){
               near_armour.push_back(armour[i]);}
       }
       //如果有近的装甲板
       if(near_armour.size()>0){
           for(int i=0;i<near_armour.size();i++){
               Point center(near_armour[i][0],near_armour[i][1]);
               cv::Mat image_roi;
               image_roi = det.image_cut(det_img,center,near_armour[i[2],rowsNumbers,colsNumbers);
               //没有更改贴纸的时候用cctag检测贴纸的圆形来最后判断装甲板
               //更改了贴纸后用caffe训练的数字模型来检测装甲板
               //bool flag = det.cctag_detect01(image_roi,save_count);
               bool flag = caffe_use(image_roi);
               if(flag){
                   circle(imgOriginal,center,5,Scalar(0,0,255),5);//红色
                   return near_armour[i];}
           }
   		//都不是的情况
           Vec3f center(2000,2000,2000);
           return center;
       }
       //如果没有比较靠近的装甲板
       else{
           for(int i=0;i<(int)armour.size();i++){
               Point center(armour[i][0],armour[i][1]);
               cv::Mat image_roi;
               image_roi = det.image_cut(det_img,center,armour[i][2],rowsNumbers,colsNumbers);
               // bool flag = det.cctag_detect01(image_roi,save_count);
               bool flag = caffe_use(image_roi);
               if(flag){
                   Point center(armour[i][0],armour[i][1]);
                   circle(imgOriginal,center,5,Scalar(0,0,255),5);//红色
                   return armour[i];
               }
           }
           //都不是的情况
           Vec3f center(2000,2000,2000);
           return center;
       }
   }
   ```

   

   

   

