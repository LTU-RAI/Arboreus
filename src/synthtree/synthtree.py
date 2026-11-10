"""
treesim.py: Functions for tree model simulation and sampling
Updated with:
 - Automatic extraction of stem height and base radius from trunk meshes
 - Detection and skipping of ground mesh
 - Multi-object trunk mesh loading and merging
 - Centerline extraction by slicing
"""

import os
import trimesh
import numpy as np
import random
from math import *

###

def get_spline_params(p0, p1, p2):
    A = np.array([ [2/(p1[0]-p0[0]), 1/(p1[0]-p0[0]), 0], \
        [1/(p1[0]-p0[0]), 2*((1/(p1[0]-p0[0]))+(1/(p2[0]-p1[0]))), 1/(p2[0]-p1[0])], \
        [0, 1/(p2[0]-p1[0]), 2/(p2[0]-p1[0])] ])
    b = np.array([3*(p1[1]-p0[1])/pow(p1[0]-p0[0],2), \
        3*( (p1[1]-p0[1])/pow(p1[0]-p0[0],2) + (p2[1]-p1[1])/pow(p2[0]-p1[0],2) ), \
        3*(p2[1]-p1[1])/pow(p2[0]-p1[0],2)])
    k = np.linalg.solve(A, b)
    return k

def get_spline_points(p_spline,kx,ky,z):
	
	x0 = p_spline[0][0]
	y0 = p_spline[0][1]
	z0 = p_spline[0][2]
	x1 = p_spline[1][0]
	y1 = p_spline[1][1]
	z1 = p_spline[1][2]
	x2 = p_spline[2][0]
	y2 = p_spline[2][1]
	z2 = p_spline[2][2]
	
	a01x = kx[0]*(z1-z0) - (x1-x0);
	a12x = kx[1]*(z2-z1) - (x2-x1);
	a01y = ky[0]*(z1-z0) - (y1-y0);
	a12y = ky[1]*(z2-z1) - (y2-y1);
	
	b01x = -kx[1]*(z1-z0) - (x1-x0);
	b12x = -kx[2]*(z2-z1) - (x2-x1);
	b01y = -ky[1]*(z1-z0) - (y1-y0);
	b12y = -ky[2]*(z2-z1) - (y2-y1);
	
	t01 = (z-z0)/(z1-z0)
	t12 = (z-z1)/(z2-z1)
	
	x01out = x0*(1-t01) + x1*t01 + np.multiply(np.multiply(t01,(1-t01)),(a01x*(1-t01) + b01x*t01))
	x12out = x1*(1-t12) + x2*t12 + np.multiply(np.multiply(t12,(1-t12)),(a12x*(1-t12) + b12x*t12))
	y01out = y0*(1-t01) + y1*t01 + np.multiply(np.multiply(t01,(1-t01)),(a01y*(1-t01) + b01y*t01))
	y12out = y1*(1-t12) + y2*t12 + np.multiply(np.multiply(t12,(1-t12)),(a12y*(1-t12) + b12y*t12))
	
	xout = np.multiply(x01out,(z < z1)) + np.multiply(x12out,(z >= z1))
	yout = np.multiply(y01out,(z < z1)) + np.multiply(y12out,(z >= z1))
	
	return (xout,yout)

###

def gen_simtree(Np=1024, stem_frac_lambda=0.3, model_params=None, tx=None, ty=None):

    # Determine number of stem points per tree
    if isinstance(stem_frac_lambda, list):
        stem_frac_lambda = stem_frac_lambda[0] + np.random.random() * (stem_frac_lambda[1] - stem_frac_lambda[0])
    Nstem = int(stem_frac_lambda * Np)
    Nfol = Np - Nstem

    if tx is None:  tx = 0
    if ty is None:  ty = 0
    tz = 0

    hr = model_params['height_range']
    h = hr[0] + (hr[1]-hr[0])*np.random.random()
    dr = model_params['diam_range']
    r = 0.5*dr[0] + 0.5*(dr[1]-dr[0])*np.random.random()

    stem_data = []
    
    # Initial spline for main stem
    p0 = [tx,ty,tz]
    sweeptopx = 2*model_params['tree_top_dist']*np.random.random()-model_params['tree_top_dist']
    sweeptopy = 2*model_params['tree_top_dist']*np.random.random()-model_params['tree_top_dist']
    tmdist = model_params['tree_mid_dist']
    
    p1 = [tx+2*tmdist*np.random.random()-tmdist+0.2*sweeptopx,ty+2*tmdist*np.random.random()-tmdist+0.2*sweeptopy,0.5*h+(0.167*h*np.random.random())]
    p2 = [tx+sweeptopx,ty+sweeptopy,h]
    p_spline = np.array([p0, p1, p2])

    kx = get_spline_params( (p_spline[0][2],p_spline[0][0]), (p_spline[1][2],p_spline[1][0]), (p_spline[2][2],p_spline[2][0]))
    ky = get_spline_params( (p_spline[0][2],p_spline[0][1]), (p_spline[1][2],p_spline[1][1]), (p_spline[2][2],p_spline[2][1]))

    zc = np.linspace(tz,h,num=20)
    (xc,yc) = get_spline_points(p_spline,kx,ky,zc)
    stem_data.append([xc,yc,zc,p_spline,kx,ky,[0,h]])

    verts = []
    faces = []
    lv = 0
    for stem in stem_data:
        (xc,yc,zc,p,kx,ky,zrange) = stem
        Ndotsl = 12
        t = np.linspace(0,2*pi,Ndotsl+1)
        t = t[:-1]
        for i in range(len(zc)):
            rn = r*(1-(zc[i]/h))
            xn = rn*np.sin(t)+xc[i]
            yn = rn*np.cos(t)+yc[i]
            for j in range(xn.shape[0]):
                verts.append([xn[j],yn[j],zc[i]])
        for i in range(len(zc)-1):
            for j in range(t.shape[0]-1):
                faces.append([Ndotsl*i+j+1+lv,Ndotsl*i+j+lv,Ndotsl*(i+1)+j+lv,Ndotsl*(i+1)+j+1+lv])
            faces.append([Ndotsl*i+lv,Ndotsl*i+t.shape[0]-1+lv,Ndotsl*(i+1)+t.shape[0]-1+lv,Ndotsl*(i+1)+lv])
        lv = len(verts)
    
    trunk_mesh = trimesh.Trimesh(vertices=verts,faces=faces)

    ###

    # Determine trunk base Z and check if Z decreases upwards
    trunk_z_min = min([trunk_mesh.bounds[0][2]])
    trunk_z_max = max([trunk_mesh.bounds[1][2]])
    flip_z = trunk_z_max < trunk_z_min  # True if Z decreases upwards

    # Sample stem points
    pts_stem = trunk_mesh.sample(Nstem)
    if flip_z:
        pts_stem[:, 2] = -pts_stem[:, 2]
    pts_stem[:, 2] -= min(pts_stem[:, 2])  # align base to 0
    pts_stem = np.concatenate((pts_stem, np.ones((Nstem, 1))), axis=1)  # class=1 for stem

    # Procedural foliage generation
    h1r = model_params["min_can_height"]
    h2r = model_params["max_can_width_height"]

    h1 = h * (h1r[0] + (h1r[1]-h1r[0]) * np.random.random())
    h2 = h * (h2r[0] + (h2r[1]-h2r[0]) * np.random.random())

    branchz1 = h1 + (h2 - h1) * np.random.random(
        np.random.randint(int(0.166 * model_params["num_branches"][0]),
                            int(0.3 * model_params["num_branches"][1]))
    )
    branchz2 = h2 + (h - h2) * np.random.random(
        np.random.randint(int(0.833 * model_params["num_branches"][0]),
                            int(0.7 * model_params["num_branches"][1]))
    )
    branchz = np.sort(np.concatenate((branchz1, branchz2)))

    xyc = []
    for stem in stem_data:
        (x,y,z,p_spline,kx,ky,zrange) = stem
        (xc,yc) = get_spline_points(p_spline,kx,ky,branchz)
        xyc.append((xc,yc))

    verts, faces = [], []
    c = 0
    rolloffh, rolloffh2 = h2, h1

    for i in range(len(branchz)):
        if (i % 10) == 0:
            gap = np.random.random()
            gap2 = (2 * np.pi - gap) * np.random.random()
        theta = 2 * np.pi * np.random.random()

        rn = r * (1 - (branchz[i] / h))
        rout = (model_params['max_can_width'] / 7.0) * np.sqrt(h - branchz[i])
        if branchz[i] < rolloffh2:
            rout = 0
        elif branchz[i] < rolloffh:
            rout -= rout * ((rolloffh - branchz[i]) / rolloffh)

        rout2 = rout + 0.4 * rout * np.random.random() - 0.2 * rout
        thick = 0.2 * rout2

        verts.append([rn * np.sin(theta) + xc[i], rn * np.cos(theta) + yc[i], branchz[i]])
        verts.append([(rout2/3)*np.sin(theta)+thick*np.cos(theta)+xc[i],
                        (rout2/3)*np.cos(theta)-thick*np.sin(theta)+yc[i],
                        branchz[i] + np.random.random()])
        verts.append([(rout2/3)*np.sin(theta)-thick*np.cos(theta)+xc[i],
                        (rout2/3)*np.cos(theta)+thick*np.sin(theta)+yc[i],
                        branchz[i] + np.random.random()])
        verts.append([(2*rout2/3)*np.sin(theta)+thick*np.cos(theta)+xc[i],
                        (2*rout2/3)*np.cos(theta)-thick*np.sin(theta)+yc[i],
                        branchz[i] + np.random.random()])
        verts.append([(2*rout2/3)*np.sin(theta)-thick*np.cos(theta)+xc[i],
                        (2*rout2/3)*np.cos(theta)+thick*np.sin(theta)+yc[i],
                        branchz[i] + np.random.random()])
        verts.append([rout2*np.sin(theta)+xc[i], rout2*np.cos(theta)+yc[i],
                        branchz[i] + np.random.random()])

        faces.append([6*c, 6*c+1, 6*c+2])
        faces.append([6*c+1, 6*c+2, 6*c+4])
        faces.append([6*c+1, 6*c+4, 6*c+3])
        faces.append([6*c+3, 6*c+4, 6*c+5])
        c += 1

    fol_mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    foliage_pts = fol_mesh.sample(Nfol)
    if flip_z:
        foliage_pts[:, 2] = -foliage_pts[:, 2]
    foliage_pts[:, 2] -= min(foliage_pts[:, 2])
    foliage_pts[:, 2] = np.clip(foliage_pts[:, 2], 0, h)
    pts_foliage = np.concatenate((foliage_pts, np.zeros((Nfol, 1))), axis=1)  # class=0

    # Join stem + foliage points
    tree_points = np.concatenate((pts_stem, pts_foliage), axis=0)

    # Concatenate all trees into a single array
    return tree_points
