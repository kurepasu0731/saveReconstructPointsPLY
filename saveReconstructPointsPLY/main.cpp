#pragma once

#include <Windows.h>
#include <opencv2\opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

//XML�t�@�C���ǂݍ���
std::vector<cv::Point3f> loadXMLfile(const std::string &fileName);

//PLY�`���ŕۑ�
void savePLY(std::vector<cv::Point3f> points, const std::string &fileName);
void savePLY_with_normal(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, const std::string &fileName);
void savePLY_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName);

//�@���x�N�g�������߂�
std::vector<cv::Point3f> getNormalVectors(std::vector<cv::Point3f> points);
//�_�E���T���v�����O
std::vector<cv::Point3f> getDownSampledPoints(std::vector<cv::Point3f> points, float size);
//�O�p���b�V���𐶐�
std::vector<cv::Point3i> getMeshVectors(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals);

int main()
{
	std::vector<cv::Point3f> reconstructPoints = loadXMLfile("reconstructPoints_camera.xml");

	//�L���ȓ_�̂ݎ�肾��(-1�͏���)
	std::vector<cv::Point3f> validPoints;
	for(int n = 0; n < reconstructPoints.size(); n++)
	{
		if(reconstructPoints[n].x != -1) validPoints.emplace_back(cv::Point3f(reconstructPoints[n].x/1000, reconstructPoints[n].y/1000, reconstructPoints[n].z/1000)); //�P�ʂ�m��
	}

	//�_�E���T���v�����O
	std::vector<cv::Point3f> sampledPoints = getDownSampledPoints(validPoints, 0.01f);

	//�@�������߂�
	std::vector<cv::Point3f> normalVecs = getNormalVectors(sampledPoints);

	//���b�V�������߂�
	std::vector<cv::Point3i> meshes = getMeshVectors(sampledPoints, normalVecs);

	//PLY�`���ŕۑ�
	savePLY_with_normal_mesh(sampledPoints, normalVecs, meshes, "reconstructPoints_camera_mesh.ply");
	//savePLY_with_normal(sampledPoints, normalVecs, "reconstructPoints_camera.ply");
	//savePLY(sampledPoints, "reconstructPoints_camera_down.ply");

	return 0;
}

// XML�t�@�C���ǂݍ���
std::vector<cv::Point3f> loadXMLfile(const std::string &fileName)
{
	//�ǂݍ��ޓ_�Q
	std::vector<cv::Point3f> reconstructPoints;
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);
	cvfs["points"] >> reconstructPoints;

	return reconstructPoints;
}


//---�ۑ��֌W---//

//PLY�`���ŕۑ�(�@���Ȃ�)
void savePLY(std::vector<cv::Point3f> points, const std::string &fileName)
{
	//�d�S�����߂�
	double cx = 0, cy = 0, cz = 0;
	for (int n = 0; n < points.size(); n++){
		cx += points[n].x;
		cy += points[n].y;
		cz += points[n].z;
	}
	cx /= points.size();
	cy /= points.size();
	cz /= points.size();

	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n", points.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	//�d�S�����_�ɂ���
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f \n", (points[n].x - cx), (points[n].y - cy), (points[n].z - cz));
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}

//PLY�`���ŕۑ�(�@������)
void savePLY_with_normal(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, const std::string &fileName)
{
	//�d�S�����߂�
	double cx = 0, cy = 0, cz = 0;
	for (int n = 0; n < points.size(); n++){
		cx += points[n].x;
		cy += points[n].y;
		cz += points[n].z;
	}
	cx /= points.size();
	cy /= points.size();
	cz /= points.size();

	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n", points.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	//�d�S�����_�ɂ���
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f %f %f %f \n", (points[n].x - cx), (points[n].y - cy), (points[n].z - cz), normals[n].x, normals[n].y, normals[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}

//PLY�`���ŕۑ�(�@������,mesh����)
void savePLY_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName)
{
	//�d�S�����߂�
	double cx = 0, cy = 0, cz = 0;
	for (int n = 0; n < points.size(); n++){
		cx += points[n].x;
		cy += points[n].y;
		cz += points[n].z;
	}
	cx /= points.size();
	cy /= points.size();
	cz /= points.size();

	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %d\nproperty list uchar int vertex_indices\nend_header\n", points.size(), meshes.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	//�d�S�����_�ɂ���
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f %f %f %f \n", (points[n].x - cx), (points[n].y - cy), (points[n].z - cz), normals[n].x, normals[n].y, normals[n].z);
	}
	//�ʏ��L�q
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}

//---Filter�֌W---//

//�@���x�N�g�������߂�
std::vector<cv::Point3f> getNormalVectors(std::vector<cv::Point3f> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//�P�ʂ�m
	}

	  // Create the normal estimation class, and pass the input dataset to it
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	  ne.setInputCloud (cloud);

	  // Create an empty kdtree representation, and pass it to the normal estimation object.
	  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  ne.setSearchMethod (tree);

	  // Output datasets
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (0.03);

	  // Compute the features
	  ne.compute (*cloud_normals);

	  std::vector<cv::Point3f> dst_normals;
	  for(int n = 0; n < cloud_normals->size(); n++)
	  {
		  //�h1.#QNAN0�h��0�ɂ���
		  if(!cvIsNaN(cloud_normals->at(n).normal_x))
			  dst_normals.emplace_back(cv::Point3f(cloud_normals->at(n).normal_x, cloud_normals->at(n).normal_y, cloud_normals->at(n).normal_z));
		  else
			  dst_normals.emplace_back(cv::Point3f(0, 0, 0));
	  }
	  return dst_normals;
}

//�_�E���T���v�����O
std::vector<cv::Point3f> getDownSampledPoints(std::vector<cv::Point3f> points, float size)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//�P�ʂ�m
	}

    // Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (size, size, size);
	sor.filter (*cloud_filtered);

	std::vector<cv::Point3f> dst_points;
	for(int n = 0; n < cloud_filtered->size(); n++)
		dst_points.emplace_back(cv::Point3f(cloud_filtered->at(n).x, cloud_filtered->at(n).y, cloud_filtered->at(n).z));

	return dst_points;
}


//�O�p���b�V���𐶐��A���b�V������Ԃ�
std::vector<cv::Point3i> getMeshVectors(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//�P�ʂ�m
		cloud_normals->push_back(pcl::Normal(normals[n].x, normals[n].y, normals[n].z));
	}
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	//���b�V������Ԃ�
	std::vector<cv::Point3i> dst_meshes;
	for(int n = 0; n < triangles.polygons.size(); n++)
	{
		dst_meshes.emplace_back(cv::Point3i(triangles.polygons[n].vertices[0], triangles.polygons[n].vertices[1], triangles.polygons[n].vertices[2])); 
	}

	return dst_meshes;
}


