
"""
Author: Athish Ram Das
Company: Center for Advanced Vehicle Systems, MSU
Description:
    SegmentationCAVS can perform semantic segmentation of images based
    on KMeans and Hierarcical Agglomerative Clustering 
"""

import math
import numpy as np
import cv2
import random
import scipy.spatial.distance as dist
import copy
from scipy import ndimage
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import sys
import argparse
 
def parse_args():
    parser = argparse.ArgumentParser(description="Clustering ",
                                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-c", "--clustering", help="Clustering method [kmeans, hac]")
    parser.add_argument("-k", "--num", help="Number of clusters")
    parser.add_argument("-v", "--vis", help="Visualize Results")
    parser.add_argument("-s", "--save", help="Save resulting segment")
    parser.add_argument("image", help="Input image")
    parser.set_defaults(clustering="kmeans")
    parser.set_defaults(vis='True')
    parser.set_defaults(num=3)
    parser.set_defaults(save='True')
    
    args = parser.parse_args()
    config = vars(args)
    return config


class Clustering:
    """
    This is class that takes in the values of image_path, clustering_method,
    number of clusters, visualize in a dict format and produce 
    clustered masks.
    Params
    TODO
    """
    
    def __init__(self, config) -> None:
        self.image_file = config['image']
        self.clustering = config['clustering']
        self.k = int(config['num'])
        self.vis = config['vis']
        self.save = config['save']

    def make_segments(self, img, idx):
        idx = np.transpose(np.uint8(idx))
        idx = idx+1
        labels = np.unique(idx)

        class Segments:
            def __init__(self, mask, img):
                self.mask = mask
                self.img = img
        segments = []
        for i in range(1, len(labels)+1):
            mask = copy.deepcopy(idx)
            mask[mask != i] = 0
            mask[mask == i] = 1
            img3 = cv2.bitwise_and(img, img, mask=mask)
            # segments[i]['img'] = np.array([i])
            segments_item = Segments(mask, img3)
            segments.append(segments_item)

        return segments

    def HAClustering(self, X):
        X = np.float32(X)
        m, n = X.shape
        plt.figure(1)

        num_clusters = m
        idx = np.arange(m)

        centroids = copy.deepcopy(X)
        cluster_sizes = np.ones(m)

        dists = dist.squareform(dist.pdist(centroids))
        np.fill_diagonal(dists, float('inf'))

        iteration = 0

        while num_clusters > self.k:
            iteration += 1
            print(iteration)
            min_dist = np.min(dists)

            i = np.where(dists == min_dist)[0][0]
            j = np.where(dists == min_dist)[0][1]

            # Make sure that i < j
            if i > j:
                t = i
                i = j
                j = t
            else:
                pass

            # temp = np.array([dists[i], dists[j]])
            # dists[i] = np.min(temp, axis=0)
            # dists = np.delete(dists, i, axis=0)
            # dists = np.delete(dists, i, axis=1)
            temp = np.array([dists[i], dists[j]])
            dists[i, :] = np.mean(temp, axis=0)
            dists[:, i] = np.mean(temp, axis=0)
            dists[j, :] = float('inf')
            dists[:, j] = float('inf')

            centroids[i] = (centroids[i] + centroids[j]) / 2
            cluster_sizes[i] += cluster_sizes[j]
            cluster_sizes[j] = 0
            centroids[j] = float('inf')
            idx[idx == idx[j]] = idx[i]

            num_clusters -= 1

        # Reindexing clusters
        u = np.unique(idx)
        for i in range(len(u)):
            idx[idx == u[i]] = i

        return idx

    def KMeansCLustering(self, X, centers=np.array([])):
        X = np.float32(X)
        m = X.shape[0]
        n = X.shape[1]

        if centers.size == 0:
            centers = np.array(random.choices(X, k=self.k))
        else:
            pass

        idx = np.zeros([m])
        plt.figure(1)

        iter = 0
        MAX_ITER = 300

        while True:
            old_idx = idx

            # Allotting clusters
            for i in range(m):
                min_dist = float('inf')
                for c in range(self.k):
                    distance = np.linalg.norm(X[i] - centers[c])
                    if distance < min_dist:
                        min_dist = distance
                        idx[i] = c


            # Updating cluster centers
            for cen in range(self.k):
                new_centre_members = []
                for i in range(m):
                    if idx[i] == cen:
                        new_centre_members.append(X[i])
                    else:
                        pass
                new_centre_members = np.array(new_centre_members)
                centers[cen] = np.mean(new_centre_members, axis=0)

            # End condition
            if np.array_equal(idx, old_idx):
                break

            # Stop early
            iter += 1
            if iter > MAX_ITER:
                break
        return idx

    def normalize_features(self, features):
        features = np.float32(features)
        featureNorm = copy.deepcopy(features)

        for i in range(features.shape[2]):
            mean = np.mean(features[:, :, i])
            std = np.std(features[:, :, i])
            featureNorm[:, :, i] = (featureNorm[:, :, i] - mean)/std

        return featureNorm

    def compute_position_color_features(self, img):
        """
        Appends the position and color features of the image in a 3D array of size 
        [height, width, 5]
        Params
        input: img -> downsized image
        output: features -> 3D array
        """
        height = img.shape[0]
        width = img.shape[1]
        features = np.zeros([height, width, 5])
        for i in range(3):
            features[:, :, i] = img[:, :, i]


        for row in range(height):
            features[row, :, 3] = row
        for col in range(width):
            features[:, col, 4] = col

        return features

    def compute_features(self, img2):
        """
        Creats a 3D array of shape [height, width, 6]
        features 3 layers of color features, 2 layer of positional features, 1 layer
        of gradient features 
        """
        img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        def gradient_x(img1):
            kernel = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
            img1 = cv2.GaussianBlur(img1, (5, 5), 0)
            img1 = np.float32(img1)
            grad_img = ndimage.convolve(img1, kernel)
            return grad_img


        def gradient_y(img1):
            kernel = np.transpose(np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]]))
            img1 = cv2.GaussianBlur(img1, (5, 5), 0)
            img1 = np.float32(img1)
            grad_img = ndimage.convolve(img1, kernel)
            return grad_img


        def gradient_mag(gx, gy):
            grad_img = np.hypot(gx, gy)
            return grad_img

        grad_x = gradient_x(img)
        grad_y = gradient_y(img)

        grad = gradient_mag(grad_x, grad_y)
        pc_features = self.compute_position_color_features(img2)

        features = np.dstack([pc_features, grad])
        return features


    def compute_segmentation(self, img, resize=1.0, normalize_features=False):
        """
        Runs kmeans or HAC based on the user input and returns segmented data
        """
        height = img.shape[0]
        width = img.shape[1]

        if resize != 1.0:
            d_height = int(height * resize)
            d_width = int(width * resize)
            img_small = cv2.resize(img, (d_height, d_width), interpolation=cv2.INTER_LINEAR)
        else:
            img_small = img

        # Compute features for small image
        features = self.compute_features(img_small)
        if normalize_features == True:
            features = self.normalize_features(features)

        # Feature is already normalized
        # Reshaping
        points = np.reshape(features, [features.shape[0]*features.shape[1], features.shape[2]])

        if self.clustering == "kmeans":
            idx = self.KMeansCLustering(points)
        elif self.clustering == "hac":
            idx = self.HAClustering(points)

        # Reshaping
        idx = np.reshape(idx, [features.shape[0], features.shape[1]])
        idx = np.transpose(idx)

        # Re-scaling
        idx = cv2.resize(idx, (height, width), interpolation=cv2.INTER_LINEAR)
        segments = self.make_segments(img, idx)

        return segments

    def showSegments(self, img, segments):
        """Shows all segments produced by the clustering algorithm"""
        grid_width = math.ceil(math.sqrt(len(segments)+1))
        grid_height = math.ceil((1+len(segments))/grid_width)

        plt.figure(1)

        plt.subplot(grid_height, grid_width, 1)
        plt.title('Original Image')
        plt.imshow(img)

        # Show each segment
        for i in range(len(segments)):
            plt.subplot(grid_height, grid_width, i+2)
            plt.title(f'Segment {i+1}')
            plt.imshow(segments[i].img)

        if self.save == 'True':
            plt.savefig(self.image_file[:-4]+'segments.png')
        plt.show()

    def run_compute_segmentation(self):
        """
        Runs the clustering and other functions based on the parsed arguments
        """
        image = cv2.imread(self.image_file)
        # cluster_method = 1
        # feature_function = ComputeFeatures

        normalize_features = True

        resize = 1.0

        segments = self.compute_segmentation(image,resize, normalize_features)
        if self.vis == 'True':
            self.showSegments(image, segments)

        return segments, image
    


if __name__ == "__main__":
    args = parse_args()
    clusterer = Clustering(args)
    segments, image = clusterer.run_compute_segmentation()