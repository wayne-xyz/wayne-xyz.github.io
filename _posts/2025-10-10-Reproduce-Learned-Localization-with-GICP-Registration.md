---
layout: post
title: "ReLL: Reproduce Learned Localization with GICP Registration"
date: 2025-10-10 16:00:00 +0000
categories: Localization
tags: [localization, map, ml, autonomous, DSM, lidar, imagery, geospatial]
---


## Intro
This post introduce the reproduction of the ICRA Paper: Evaluating Global Geo-alignment for Precision Learned Autonomous Vehicle Localization using Aerial Data( https://arxiv.org/abs/2503.13896). Mainly talk about the detail the paper did not mentions, including the data preparing, data outline, also mentions the chanllengings and solution I have faced. 

### Core 
GICP: (https://www.roboticsproceedings.org/rss05/p21.pdf) is brought to be a method for resgistration, and the embeding encoder model training and vali by translation metircs. It approved the GICP included can improve high resulotion localization in the imagery coordinate. 

Learned Localization: Train encoder for V&M, sliding the embeding image to measure the 

Factor graph: Because this original author managered the lots' of other data to gain the better learned localizaiton, they build the factor graph for better fine-tuning and long term goal. In my case i simplified this pipeline and 

### Data 
In my reproduction I use the argoverse-2 data https://www.argoverse.org/av2.html. which contains lidar points sample and the translate method from city frame to UTM. Which will align with the data from the open data for DSM and Imagery , in my case I focusing on the city us Austin, TX.
Bexar & Travis Counties Lidar | 2021 https://data.geographic.texas.gov/collection/?c=447db89a-58ee-4a1b-a61f-b918af2fb0bb
Capital Area Council of Governments Imagery | 2022  :https://data.geographic.texas.gov/collection/?c=a15f67db-9535-464e-9058-f447325b6251



## Data Praperation and GICP

### Preview the data 
1st i have check several potential dataset which contains the lidar and localization data , compaerd the waymo, Pit30M ,toronto_3d. they are not match the requriement and goal and have the generalization potientility .


### GICP Veri

