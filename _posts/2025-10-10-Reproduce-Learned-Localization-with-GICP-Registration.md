---
layout: post
title: "ReLL: Reproduce Learned Localization with GICP Registration of Lidar&DSM"
date: 2025-10-10 16:00:00 +0000
categories: Localization
tags: [localization, map, ml, autonomous, DSM, lidar, imagery, geospatial]
---


## Intro
This post introduce the reproduction of the ICRA Paper: Evaluating Global Geo-alignment for Precision Learned Autonomous Vehicle Localization using Aerial Data( https://arxiv.org/abs/2503.13896). This post will mainly talk about the detail the paper did not mentions, including the data preparing, data outline, also mentions the chanllengings and solution I have faced. 

Original paper mention the Geo-alignment evalution 

### Core 
GICP: (https://www.roboticsproceedings.org/rss05/p21.pdf) is brought to be a method for resgistration, and the embeding encoder model training and vali by translation metircs. It approved the GICP included can improve high resulotion localization in the imagery coordinate. 

Learned Localization: Train encoder for V&M, sliding the embeding image to measure the similartiy within a search window. Define the cost volume to measure the cross-correlation,   and stack the shift to measure the shift capcity or vibrate range . using both the pixel level offset and subpixel offset by Gaussion fit to gain the presice peak.

Factor graph: Because original author managered the lots' of other data to gain the better learned localizaiton, they build the factor graph for better fine-tuning and long term goal, the GICP is only one factor. In my case i simplified this pipeline and only applied the GICP alignment and w/ wo/ to compaere them. 

### My Data 
In my reproduction I use the argoverse-2 data https://www.argoverse.org/av2.html. which contains lidar points sample and the translate method from city frame to UTM. Which will align with the data from the open data for DSM and Imagery , in my case I focusing on the city us Austin, TX.
Bexar & Travis Counties Lidar | 2021 https://data.geographic.texas.gov/collection/?c=447db89a-58ee-4a1b-a61f-b918af2fb0bb
Capital Area Council of Governments Imagery | 2022  :https://data.geographic.texas.gov/collection/?c=a15f67db-9535-464e-9058-f447325b6251  , resolution 0.3 



## Data Praperation and GICP

### Preview the data 
1st i have check several potential dataset which contains the lidar and localization data , compaerd the waymo, Pit30M ,toronto_3d. they are not match the requriement and goal and have the generalization potientility .


### GICP Veri

- minimal verfificaiotn from one set + pertube offset to know about the effiency of the GICP 

- 3d cloud points shape consistency 

- self frame Coordinate




## Embeding Train and measure

### Train and verif loop :

1. two embeding ,  pertueb 
2. overlap , sliding within serach window 
3. each shift Δx,Δy  , correlation value - build the cost map(  cost volunms) in a stack
4. get the peak , create the 5x5 cell around it , and build the gaussian surface .
5. Gaussian mean, fractional-pixel offset .  e.g 2.31px right 1.1px up ...
6. Gaussian covariance , sharper peak(lower uncertainty )
7. 

### Parameter define, in my case i have tried several parameters which not explicited in the paper,  
- upsampling (for imagery )
search window:1m, 
v-m submap size: 32mx32m , upsampling to the 256x256pixles, embedding resulotion : 0.125m
same pertube +-1m, +-3deg
? gap  ?embedding pixel size differ from the original size 

- not explicit the cnn encoder 


reproduce 


Befroe i train the whole progress, this model train is not plain model , it will include the search windows, guassian surface and the mean+ varirance  , 