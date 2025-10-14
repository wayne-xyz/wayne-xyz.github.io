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


### what the GICP applied on which type of the data, since the spin and the DSM might looks different visually 

- minimal verfificaiotn from one set + pertube offset to know about the effiency of the GICP 

- 3d cloud points shape consistency 

- self frame Coordinate

- revisite loop clousure missing in my case , 


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


### Training: learned LiDAR ↔ map localizer

What I’m building

A learned LiDAR ↔ map (DSM + imagery) localizer. Two tied CNN encoders (shared weights) produce 128-D embeddings for LiDAR and map patches. We cross-correlate LiDAR embeddings against shifted map embeddings over a 2D search window to estimate translation.

How training works (short)

- Synthetic labels via perturbation: during training we randomly shift the LiDAR embedding relative to the map embedding. The applied shift (in embedding pixels) is the ground-truth offset.
- Cost volume: for every integer shift (dx, dy) inside the search window we compute a correlation (similarity) score and stack those into a 2D cost map (cost volume slice).
- Prediction: pick the argmax location in the cost map (discrete grid peak) as the estimated offset.
- Loss: compute the difference between predicted and ground-truth offsets, converting embedding pixels to meters using: meters = (embedding px) × (encoder stride) × (map resolution). Rotation search is disabled in these runs (max_rotation_deg = 0).

Key assumptions and units

- All perturbations, search steps and the cost volume grid are expressed in embedding pixels (the encoder's downsampled grid).
- Reported metrics are in meters. Use the conversion above to translate between embedding pixels and meters.

Observations and concerns

- Quantization / no sub-pixel estimator: using a hard argmax on a discrete embedding grid quantizes predictions to integer embedding pixels. This creates a natural error floor: even with near-perfect features you typically see residual error on the order of ~0.25–0.5 embedding px, which in the author's setup often corresponds to ~0.5–1.0 m.
- No rotation search: with max_rotation_deg = 0 we don't estimate orientation, so θ metrics will be near zero but not informative for rotation performance.
- Loss plateau: training loss sometimes stalls around ~0.6–1.2 m. This magnitude matches the expected quantization floor plus occasional mismatches in feature alignment between modalities.

Why the loss can't go below ~0.4 (yet)

With a hard discrete argmax the estimator cannot recover offsets between grid cells. Even if the model produces a very sharp, correct peak, the prediction is rounded to the nearest embedding pixel. Without a local sub-pixel refinement (soft-argmax, Gaussian fit, quadratic interpolation, or a small regression head that refines the peak), there is an irreducible error floor determined by the embedding stride and map resolution.

Short checklist of simple improvements to reduce the floor

- Add a sub-pixel peak estimator: soft-argmax, local quadratic/Gaussian fit around the peak, or a small NN head that regresses fractional offsets from the local cost patch.
- Increase embedding resolution (smaller stride or upsample before correlation) to reduce quantization step in meters (tradeoff: compute and memory).
- Enable a small rotation search or predictor if heading errors are relevant.
- Improve cross-modal feature alignment (data augmentation, contrastive losses, or additional supervision) to reduce mismatches that worsen the plateau.

These changes are low-risk and can be tested incrementally to see which gives the largest practical drop in meter-level error.