---
layout: post
title: "ReLL: Reproduce Learned Localization with GICP Registration of Lidar&DSM"
date: 2025-10-10 16:00:00 +0000
categories: Localization
tags: [localization, map, ml, autonomous, DSM, lidar, imagery, geospatial]
---


## 1 Intro
This post introduce the reproduction of the ICRA Paper: Evaluating Global Geo-alignment for Precision Learned Autonomous Vehicle Localization using Aerial Data( https://arxiv.org/abs/2503.13896). This post will mainly talk about the detail the paper did not mentions, including the data preparing, data outline,  model design ,train progress. also mentions the chanllengings and solution I have faced. 

All data pipeline method , training code open source  : 



### 1.1 Outcome
- Implemation the data pipeline including GICP registration, and model trianing progress, prove the GICP registration gain good result. 
- Experiment several loss funciotn contrscutrion , softmax, improved gaussian fit method. at the 0.2m resolution dataset gain rms 0.1m level translation acurracy.
- Explore the edge case mention in the original paper. Propose a factor relate with the lidar point fillment rate reason. 

### 1.2 Core of original paper
GICP: (https://www.roboticsproceedings.org/rss05/p21.pdf) is brought to be a method for resgistration, and the embeding encoder model training and vali by translation metircs. It approved the GICP included can improve high resulotion localization in the imagery coordinate. 

Learned Localization: Train encoder for V&M, sliding the embeding image to measure the similartiy within a search window. Define the cost volume to measure the cross-correlation,   and stack the shift to measure the shift capcity or vibrate range . using both the pixel level offset and subpixel offset by Gaussion fit to gain the presice peak.


### 1.3 My Data 
In my reproduction I use the argoverse-2 data https://www.argoverse.org/av2.html. which contains lidar points sample and the translate method from city frame to UTM. Which will align with the data from the open data for DSM and Imagery , in my case I focusing on the city us Austin, TX.
Bexar & Travis Counties Lidar | 2021 https://data.geographic.texas.gov/collection/?c=447db89a-58ee-4a1b-a61f-b918af2fb0bb
Capital Area Council of Governments Imagery | 2022  :https://data.geographic.texas.gov/collection/?c=a15f67db-9535-464e-9058-f447325b6251  , resolution 0.3047 

## 2 Gaussian fit method 


in my trainning progress i applied the softmax probabilities  method for loss function to Backpropagating the loss update the model . But the guassian fit only for the infer. 
On the mix dataset, i use the guassian fit to perform the refine position solve gain better result than softmax reuslt.

- Extract 3×3 patch centered on the discrete peak and clamp tiny values.
- Take log of pixel values → Gaussian amplitude becomes a quadratic surface.
- Use neighbor values to estimate first and second derivatives.
- Solve for stationary point: dx = −(d/dx) / (d2/dx2), dy = −(d/dy) / (d2/dy2).
- Clamp offsets to a safe range and add to integer center → sub‑pixel (x,y).
- Optionally sample score via bilinear interpolation at refined (x,y).



| **Method** | **RMS Errors (m)** | **P50 (Median) (m)** | **P99 (99th Percentile) (m)** | **Mean Distance (m)** | **Max Distance (m)** |
|:------------|:-------------------|:----------------------|:-------------------------------|:----------------------|:--------------------:|
| **Softmax Refinement** | **X:** 0.2273<br>**Y:** 0.2637<br>**Dist:** 0.3481 | **X:** 0.0711<br>**Y:** 0.1161<br>**Dist:** 0.1753 | **X:** 1.0376<br>**Y:** 1.0816<br>**Dist:** 1.2508 | 0.2498 | 1.6772 |
| **Gaussian Peak Refinement** | **X:** 0.1907<br>**Y:** 0.2222<br>**Dist:** 0.2928 | **X:** 0.0690<br>**Y:** 0.1154<br>**Dist:** 0.1701 | **X:** 0.9560<br>**Y:** 1.0050<br>**Dist:** 1.2029 | 0.2090 | 1.6772 |

Based on my data the guassian fit method gain better converge than softmax, but because of undiffenretiable guassian fit we can not applied it into the model training .

![Position Refinement Comparison](/img/Position%20Refinment%20Comparison.png)
Fig1. comapre 3 refined cross-corralation peak finding result at the mix dataset 

![Compare Peak Finding Method](/img/Comapre%20peak%20finding%20method.png)
Fig2. compare 3 refined peak finding method, most time gaussain fit method can gain equaivlant result as softmax 

![Compare Peak Finding Method 2](/img/Comparepeakfindingmethod2.png)
Fig3. sometime the guassian fit method gains better result

## 3 Fillment rate

In the original paper describe the situaiton is when under the 

I have tried two resolution plan plan1 is original imagery resoltuoin , and plan2 is the following the paper's method upsampling to 0.2. we gain better result at the upsampling method. It's high related with the fillment rate I will intro more below. 

| Plan  | Resolution (m) | Raster Size | Range (m) | x_rms (m, UTM) | y_rms (m, UTM) | θ (deg) |
|--------|----------------|-------------|------------|----------------|----------------|----------|
| Plan 1 | 0.3047         | 329 × 329   | 100 × 100  | 0.483          | 0.423          | 0.7083   |
| Plan 2 | 0.2            | 150 × 150   | 30 × 30    | 0.104          | 0.119          | 0.4480   |


