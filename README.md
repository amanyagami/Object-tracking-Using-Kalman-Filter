# Object Tracking using Kalman Filters 

<br>
<font size="3">
                          Aman Singh <br>
</font>

#  Kalman Filter

<font size="4">Kalman filter is an algorithm that produces estimates of an unknown variable by using a series of data taken over time along with statistical noise and inaccuracies.Kalman filter uses joint probablilty distribution over the variables for each time frame.It produces better estimates than those algorithms which are based on a single measurement. 

Today kalman filters are used in guidance, navigation of vehicles such a aircraft, spacecraft and dynamically positioned ships. 
    
   
</font>

# Basics and application in 1-D

![](1.jpeg)

<font size="4">
A variable whose change doesn't have an uncertainity and error can predicted precisely. Hence kalman filter is used to predict such variables which have errors in measurement and prediction.

The Kalman filter represents all distributions by Gaussian distribution.
Mean(μ) and Variance(σ2) representing a Gaussian distibution is taken as the best estimate for representing the probality distribution for the measured and estimated position of the car along the x-axis for Kalman Filter.

Kalman filters has two major iterative steps :
    
1.Predict    
    
2.Update
</font>

![](1.png)



<br>
<font size="4">
        
A 1-D Kalman Filter is used to track an object moving along the x-axis.
For example a car moving along a road with it's position being measured and estimated.
    
We assume that there is Gaussian noise ( 𝑢  ,  𝑟2 ) in measurement of the position of the car. Hence the mean and variance of the car changes as

</font>

![e1](https://latex.codecogs.com/png.latex?%5Cmu%5E%7B%27%7D%20%3D%20%5Cmu&plus;u)

![e2](https://latex.codecogs.com/png.latex?%5Csigma%5E%7B%27%7D%20%3D%20%5Csigma%5E2%20&plus;%20r%5E2)

<br>

<font size="4">
 The Update step takes in the predicted postion of the car in terms of the mean ,𝜈  and variance ,𝑟2 of the Gaussian distribution representing it and updates the position based on the measured position represnted by it's mean,𝜇 and variance,𝜎2 

</font>

![e3](https://latex.codecogs.com/png.latex?%5Cmu%5E%7B%27%7D%20%3D%20%5Cfrac%7B%5Cmu%20r%5E2%20&plus;%20%5Cnu%5Csigma%5E2%7D%7Br%5E2%20&plus;%20%5Csigma%5E2%7D)

![e4](https://latex.codecogs.com/png.latex?%5Csigma%5E%7B%27%7D%20%3D%20%5Cfrac%7B1%7D%7B%5Cfrac%7B1%7D%7Br%5E2%7D&plus;%5Cfrac%7B1%7D%7B%5Csigma%5E2%7D%7D)

# Kalman Filter Equations  For 1-D model

![](3.png)

<br>
<font size="4">
 
 *  ![](https://latex.codecogs.com/png.latex?%5Cbf%7BPredict%7D%24%20%24%20%5Cbf%7BEquations%20%7D) : To find the current state using the previous state.
    <br>
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D) : Predicted state estimate<br>
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BP_k%7D): Predicted error covariance<br>
      
<br><br>
    
* ![](https://latex.codecogs.com/png.latex?%5Cbf%7BCorrector%7D%24%20%24%20%5Cbf%7BEquations%7D): To improve the curent estimated state by incorporating a new measurement into the model
    <br>
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7Bv_t%7D) : Measurement residual<br>
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BK_t%7D) : Kalman gain
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D) : Updated state estimate
   
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BP_t%7D) : Updated error covariance 

    
    
<br><br>

</font>

# Predict() Equations 

<br>
<font size="4">
For an object whose postion on the x-axis varies as: 
  X(t)= t + 0.5t^2
  
The state estimate is given by
  
![](https://latex.codecogs.com/png.latex?X_t%20%3D%20A_t%20X_%7Bt-1%7D%20&plus;%20Bu_t)

the error covariance matrix  is given by 
  
![](https://latex.codecogs.com/png.latex?P_k%20%3D%20E%5B%28X%5E%7Bacutal%7D_t%20-X_t%29%28X%5E%7Bactual%7D_t%20-X_t%29%5ET%5D)
    
![](https://latex.codecogs.com/png.latex?%7BP%7D_k%20%3D%20A%20%7BP%7D_%7Bk-1%7DA%5ET&plus;%20%7BQ%7D)


    
 Updating ![](https://latex.codecogs.com/png.latex?X_t)
  
![](https://latex.codecogs.com/png.latex?X_t%20%3D%20X_%7Bt-1%7D%20&plus;%20%5Cdot%7BX_%7Bt-1%7D%7D%5CDelta%20t%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%5Cdot%7B%5Cdot%7BX_t%7D%7D%7B%5CDelta%20t%7D%5E2)
    
And ![](https://latex.codecogs.com/png.latex?%5Cdot%7BX_t%7D) is 
  
![](https://latex.codecogs.com/png.latex?%5Cdot%7BX_t%7D%20%3D%20%5Cdot%7BX_%7Bt-1%7D%7D%20&plus;%20%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D%5CDelta%20t)
    
And ![](https://latex.codecogs.com/png.latex?%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D)(acceleration of the car at time t-1) is assumed constant
    
    
The state vector
![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D%3D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20%5Cdot%7BX_t%7D%5Cend%7Bbmatrix%7D%3D%20%5Cbegin%7Bbmatrix%7D%20X_%7Bt-1%7D%20&plus;%20%5Cdot%7BX_%7Bt-1%7D%7D%5CDelta%20t%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%5Cdot%7B%5Cdot%7BX_t%7D%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5Cdot%7BX_%7Bt-1%7D%7D%20&plus;%20%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D%5CDelta%20t%20%5Cend%7Bbmatrix%7D)


On Simplifying


![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D%3D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20%5Cdot%7BX_t%7D%5Cend%7Bbmatrix%7D%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%20%5CDelta%20t%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20X_%7Bt-1%7D%20%5C%5C%20%5Cdot%7BX_%7Bt-1%7D%7D%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5CDelta%20t%5Cend%7Bbmatrix%7D%20%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D)

Comparing the above equation with kalman equations and matching the coefficents

![](https://latex.codecogs.com/png.latex?A%20%3D%5Cbegin%7Bbmatrix%7D%201%20%26%20%5CDelta%20t%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D)
![](https://latex.codecogs.com/png.latex?B%20%3D%20%5Cbegin%7Bbmatrix%7D%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5CDelta%20t%5Cend%7Bbmatrix%7D)

Here acceleration can be thought of as the control input and B is the matrix that maps it to position and velocity. 
</font>


# Update () Equations

<br>
<font size="4">
    
![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20H_tX_t%20&plus;%20%5Cnu_t)   
    
![](https://latex.codecogs.com/png.latex?-%3E%5Cnu_t%20%3D%20Y_t-H_tX_t........%20%28a%29)
    
The Updated state estimate is 
    
![](https://latex.codecogs.com/png.latex?X_t%20%3D%20X_t%20&plus;%20K_t%5Cnu_t)<br>

Substituting the equation (a)
    
![](https://latex.codecogs.com/png.latex?X_t%20%3D%20X_t%20&plus;%20K_t%28Y_t-H_tX_t%29)

![](https://latex.codecogs.com/png.latex?K_t)is the ![](https://latex.codecogs.com/png.latex?%5Cbf%7BKalman%7D%24%20%24%5Cbf%7B%20Gain%7D).<br>
    

![](https://latex.codecogs.com/png.latex?K_t) is given by
    
![](https://latex.codecogs.com/png.latex?%7BK%7D_t%20%3D%20%5Cfrac%7B%7BP%7D_tH%5ET%7D%7BH%7BP%7D_tH%5ET&plus;%20%7BR%7D%20%7D)
    
 R is the measurement covariance matrix.
    
The updated error covariance matrix is 
    
![](https://latex.codecogs.com/png.latex?P_t%20%3D%20%28I%20-%20K_tH_t%29P_t)
    
In case of a car we can only measure its position hence $Y_t$ is   
    
![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20X_t%20&plus;%20%5Cnu_t)
    
![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20%5Cdot%7BX_t%7D%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cnu_t)
    
![](https://latex.codecogs.com/png.latex?-%3EY_t%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%5Cend%7Bbmatrix%7D%5Cbf%7BX_t%7D%20&plus;%20%5Cnu_t)
    
Comparing the coefficients with the kalman equation 
    
![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%5Cend%7Bbmatrix%7D%5Cbf%7BX_t%7D%20&plus;%20%5Cnu_t)
    
![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20H_tX_t%20&plus;%20%5Cnu_t)
    
![](https://latex.codecogs.com/png.latex?-%3EH%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%5Cend%7Bbmatrix%7D)
 
</font>

##  Measurement noise covariance  R  and Process noise covariance Q

<br>
<font size="4">
    
If ![](https://latex.codecogs.com/png.latex?%5Csigma_x%24%20and%20%24%5Csigma_%7B%5Cdot%7Bx%7D%7D)are the standard deviations of the position and the velocity then the  Process Noise Covariance Q is 
    
![](https://latex.codecogs.com/png.latex?Q%3D%5Cbegin%7Bbmatrix%7D%20%7B%5Csigma_x%7D%5E2%20%26%20%5Csigma_x%5Csigma_%7B%5Cdot%7Bx%7D%7D%5C%5C%20%5Csigma_x%5Csigma_%7B%5Cdot%7Bx%7D%7D%20%26%20%5Csigma_%7B%5Cdot%7Bx%7D%7D%5E2%5Cend%7Bbmatrix%7D)

If ![](https://latex.codecogs.com/png.latex?%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D) is the standard deviation of acceleration we can write
![](https://latex.codecogs.com/png.latex?%5Csigma_x%20%3D%20%5Cfrac%7B%5CDelta%20t%5E2%7D%7B2%7D%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D)
![](https://latex.codecogs.com/png.latex?%5Csigma_%7B%5Cdot%7Bx%7D%7D%20%3D%20%5CDelta%20t%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D)
    
Thus 
![](https://latex.codecogs.com/png.latex?Q%3D%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B4%7D%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%5C%5C%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%26%20%5CDelta%20t%5E2%5Cend%7Bbmatrix%7D%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D%5E2) 

And R is just a scalar as we measure only one variable ,that is position.
   
 
![](https://latex.codecogs.com/png.latex?R%20%3D%20%5Csigma_m%5E2)
    
Q and R update the error covariance matrix ![](https://latex.codecogs.com/png.latex?P_t)

</font>


![](1d1.png)
![](1d2.png)
![](1d3.png)
![](1d4.png)

# Object detection in 2-D

<br>

<font size="4">
Blob detection is used to encirlce the object in the video which is used to test the 2D filter.
Blob detection converts the video into grayscale, detects the edjes and sets a threshold to find contours. Then the min radius cirlce is calculate which encircles the contour and it provides a centroid.

Here we use a video of a ball bouncing off the screen to test our kalman filter and blob detection.
    
</font>

![](detect.png)

# Kalman Filter Equations for 2-D model
 
<br>
<font size="4">
The only major change in 2-D motion from 1-D is variable for state vector.
All the Kalman equations remain the same and apply in both X and Y direction.
Considering $X$ and $Y$ to be the concerned variables denoting the position the state vector should look like
    
![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D%3D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20Y_t%20%5C%5C%5Cdot%7BX_t%7D%5C%5C%20%5Cdot%7BY_t%7D%5Cend%7Bbmatrix%7D)
    
Subsituting the Kalman 1-D equations
    
![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D%3D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20Y_t%20%5C%5C%5Cdot%7BX_t%7D%5C%5C%20%5Cdot%7BY_t%7D%5Cend%7Bbmatrix%7D%3D%20%5Cbegin%7Bbmatrix%7D%20X_%7Bt-1%7D%20&plus;%20%5Cdot%7BX_%7Bt-1%7D%7D%5CDelta%20t%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%5Cdot%7B%5Cdot%7BX_t%7D%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20Y_%7Bt-1%7D%20&plus;%20%5Cdot%7BY_%7Bt-1%7D%7D%5CDelta%20t%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%5Cdot%7B%5Cdot%7BY_t%7D%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5Cdot%7BX_%7Bt-1%7D%7D%20&plus;%20%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D%5CDelta%20t%20%5C%5C%20%5Cdot%7BY_%7Bt-1%7D%7D%20&plus;%20%5Cdot%7B%5Cdot%7BY_%7Bt-1%7D%7D%7D%5CDelta%20t%20%5Cend%7Bbmatrix%7D)

On simplifying

![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D%3D%5Cbegin%7Bbmatrix%7D%20X_t%20%5C%5C%20Y_t%20%5C%5C%5Cdot%7BX_t%7D%5C%5C%20%5Cdot%7BY_t%7D%5Cend%7Bbmatrix%7D%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%5CDelta%20t%20%26%200%5C%5C%200%20%26%201%20%26%200%20%26%5CDelta%20t%20%5C%5C%200%20%260%20%261%20%260%20%5C%5C%200%20%26%200%20%26%200%20%26%201%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20X_%7Bt-1%7D%20%5C%5C%20Y_%7Bt-1%7D%5C%5C%5Cdot%7BX_%7Bt-1%7D%7D%5C%5C%5Cdot%7BY_%7Bt-1%7D%7D%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%26%200%20%5C%5C0%20%26%20%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5CDelta%20t%20%26%200%20%5C%5C0%20%26%20%5CDelta%20t%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%5Cdot%7B%5Cdot%7BX_%7Bt-1%7D%7D%7D%5C%5C%5Cdot%7B%5Cdot%7BY_%7Bt-1%7D%7D%7D%5Cend%7Bbmatrix%7D)

On comparing the Coefficients from the Kalman equation of state we get:
    
![](https://latex.codecogs.com/png.latex?A%20%3D%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%5CDelta%20t%20%26%200%5C%5C%200%20%26%201%20%26%200%20%26%5CDelta%20t%20%5C%5C%200%20%260%20%261%20%260%20%5C%5C%200%20%26%200%20%26%200%20%26%201%5Cend%7Bbmatrix%7D%20%3BB%3D%5Cbegin%7Bbmatrix%7D%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%26%200%20%5C%5C0%20%26%20%5Cfrac%7B1%7D%7B2%7D%7B%5CDelta%20t%7D%5E2%20%5C%5C%20%5CDelta%20t%20%26%200%20%5C%5C0%20%26%20%5CDelta%20t%20%5Cend%7Bbmatrix%7D)
    
Since only measurement available is position H is 
    
![](https://latex.codecogs.com/png.latex?H%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%20%26%200%20%5C%5C%200%20%26%201%20%260%26%200%20%5Cend%7Bbmatrix%7D)

    
Process Noise covariance Q and Measurement noise covariance R  
    
![](https://latex.codecogs.com/png.latex?Q%3D%5Cbegin%7Bbmatrix%7D%20%7B%5Csigma_x%7D%5E2%20%26%200%20%26%5Csigma_x%5Csigma_%7B%5Cdot%7Bx%7D%7D%26%200%5C%5C0%26%20%7B%5Csigma_y%7D%5E2%20%26%200%20%26%5Csigma_y%5Csigma_%7B%5Cdot%7By%7D%7D%5C%5C%5Csigma_x%5Csigma_%7B%5Cdot%7Bx%7D%7D%20%26%200%20%26%5Csigma_%7B%5Cdot%7Bx%7D%7D%5E2%20%26%200%5C%5C%200%26%5Csigma_y%5Csigma_%7B%5Cdot%7By%7D%7D%20%26%200%20%26%5Csigma_%7B%5Cdot%7By%7D%7D%5E2%20%5Cend%7Bbmatrix%7D)
    

![](https://latex.codecogs.com/png.latex?%5Csigma_x%20%3D%20%5Cfrac%7B%5CDelta%20t%5E2%7D%7B2%7D%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D)
  
![](https://latex.codecogs.com/png.latex?%5Csigma_%7B%5Cdot%7Bx%7D%7D%20%3D%20%5CDelta%20t%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D)
    
Thus 
![](https://latex.codecogs.com/png.latex?Q%3D%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B4%7D%20%26%200%20%26%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%26%200%5C%5C%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B4%7D%20%26%200%20%26%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%5C%5C%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%260%20%26%5CDelta%20t%5E2%20%26%200%20%5C%5C%200%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%260%20%26%5CDelta%20t%5E2%20%5Cend%7Bbmatrix%7D%20%5Csigma_%7B%5Cdot%7B%5Cdot%7Bx%7D%7D%7D%5E2)
    
The measurement noise R in 2-D becomes

![](https://latex.codecogs.com/png.latex?R%20%3D%20%5Cbegin%7Bbmatrix%7D%5Csigma_x%5E2%20%26%200%20%5C%5C%200%20%26%5Csigma_y%5E2%5Cend%7Bbmatrix%7D)
    
![](https://latex.codecogs.com/png.latex?P_t)
</font>



![](2d1.png)

![](2d2.png)

![](2d3.png)

![](2d4.png)

#  Multiple Object tracking 

<br>
<font size="4">
    
Multiple Object tracking allows the model to track and predict the position for multiple objects. So in order to differentiate between any two objects in the same frame we need to assign them an ID once it is detected.
    
Then the object is detected again in the next frame. We find the object with the same Id in the next frame by using the minimum weight matching which uses euclidian distance between two coordinate as weight.

The below diagram gives an idea about the same.  
</font>



![](bipartite.png)




    
## State Update
<br>

<font size="4">
The state of the system evolves with time as
    
![](https://latex.codecogs.com/png.latex?X_t%20%3D%20A_t%20X_%7Bt-1%7D%20&plus;%20Bu_t%20&plus;%20w_t)
    
Where ,<br>
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BX_t%7D) : State Vector <br>
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BA_t%7D): The State Transition Matrix. <br>
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7Bu_t%7D) : The Vector with control inputs.<br>
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BB_t%7D) : The Control Input matrix that maps the control input to the coresponding State Vector <br>represntation.
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7Bw_t%7D):  Noise with a covariance Q. 


</font>

## System Measurement
<font size="4">

![](https://latex.codecogs.com/png.latex?Y_t%20%3D%20H_tX_t%20&plus;%20%5Cnu_t)
    
Where ,<br>
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BY_t%7D) : Vector of measurements <br>
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7BH_t%7D): Matrix that maps $X_t$ to measurement space<br>          
    
   * ![](https://latex.codecogs.com/png.latex?%5Cbf%7B%5Cnu_t%7D) :  Measurement Noise $(0, R)$ <br>
    
Note :![](https://latex.codecogs.com/png.latex?%5Ctextbf%7Bw%7D_k%20and%20%5Ctextbf%7Bv%7D_k) represent the process noise vector with the covariance Q and the measurement noise vector with the covariance R, respectively. They are assumed statistically independent Gaussian noise with the normal probability distribution.
    p(w) = N(0,Q)
    p(![](https://latex.codecogs.com/png.latex?%5Cnu))=N(0,R)
    
</font>

# The Hungarian Algorithm 

The Hungarian Algorithm helps to associate one object from one frame to another based on a score. The score used here is Convolution Cost.

<br>
<font size="4">
The bipartite graph can be represnted as an adjacency matrix as shown
    
![](https://latex.codecogs.com/png.latex?Adjacency%20%3D%5Cbegin%7Bbmatrix%7D%20dist_%7B11%7D%20%26%20dist_%7B12%7D%20%26%20dist_%7B13%7D%20%26%20....%20%26%20dist_%7B1M%7D%20%5C%5C%20dist_%7B21%7D%20%26%20dist_%7B22%7D%20%26%20dist_%7B23%7D%20%26%20....%20%26%20dist_%7B2M%7D%20%5C%5C.%20%26%20.%20%26%20.%20%26%20.....%20%26%20.%20%5C%5C.%20%26%20.%20%26%20.%20%26%20.....%20%26%20.%20%5C%5C.%20%26%20.%20%26%20.%20%26%20.....%20%26%20.%20%5C%5C%20dist_%7BN1%7D%20%26%20dist_%7BN2%7D%20%26%20dist_%7BN3%7D%20%26%20....%20%26%20dist_%7BNM%7D%5Cend%7Bbmatrix%7D)

Now the Hungarian Algorithm helps us arrive at the final optimal assignment using a few steps that involve row and column reduction on the adjacency matrix. 
    
The Hungarian Algorithm can be captured in the following steps: 
    <br>
   *  : Find the smallest element in each row and subtract every elemnt in that row with that element.
    <br>
   *  : Draw lines so that all the zeros are covered with minimal number of lines 
    <br>
   *  : If number of line is N then algorithm is done and all we need to do is generate assignments based on the zero elements.
    <br>
   *  :Find the smallest entry not covered by any line. Now subtract this from every uncovered algorithm and add it to the  elements that have the horizontal and vertical lines cross each other. Now try from step 3 again. <br>
<br><br>
    




## Multiple Object tracking

  <br>
<font size="4"> 
 
 The steps in the process are in the image below.
</font>

![](MAIN.png)
<font size="4"> 
    The Detector and ObjectTracker are called here to detect objects in the video input. The code can be understood using this diagram.For the object detection this kalman filter used pretrained weights based on DNN using opencv.
    <br>
    Using a real video of road as input the kalman filter tracks the multiple ojects in the video.
 </font>
