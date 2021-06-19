// Algorithm has 2 parts:
// 1) Group the 3-D points into clusters.
// 2) Filter out irrelevant clusters to finally contain only those clusters corresponding to trees.
// 		Three stages of filtering are done in this step:
//  (a) Retain only clusters of size between `cluster_size_low_threshold` & `cluster_size_high_threshold` & discard the rest
//  (b) Calculate the median cylinder radius of each cluster & discard those clusters with radius > `tree_radius_thresh`
//  (c) Discard the clusters which reside completely inside the median cylinder
// 		  (Because, planar surfaces (of a facade/grass/vehicle) have large radius & hence, more likely than not, they completely reside inside the median cylinder.)
// 		  (Whereas, curved surfaces (like clusters of foliage/trunk/(entire tree)) have smaller radius & hence, clusters comprising curved surfaces tend to protrude out of their median cylinder)
//        (This was able to remove: portions of facades(Example: Compare WRLs in generated_wrl/med_cylinder/enclose/3_aj/)
//        (						    current poles, vehicles(Example: Compare WRLs in generated_wrl/med_cylinder/enclose/3_ak/))

// Usage: g++ -o code final_tree_segmenter_latest_v2.cpp
// Usage: ./code environment_name number_of_3Dpoints_in_the_current_environment
// Example Usage: ./code 2_ac 100000
// OUTPUTS:
// Output of 1st part of algo (generated only if do_clustering == 1):
// 		The wrl files generated after 1st part of algo are generated in `generated_wrl/2_ac` directory. I've manually moved the generated wrl files into `generated_wrl/2_ac/z_range_0.2_0.35_proxi_0.7` after code execution. Could have written code for that. Will do that as final touch-ups.
// Output of 2nd part of algo:
// 		Output of 2-(a) along with median cylinders of the obtained clusters is present in "generated_wrl/med_cylinder/2_ac/median_cylinder.wrl"
// 		Output of 2-(b) along with median cylinders of the obtained clusters is present in "generated_wrl/med_cylinder/2_ac/median_cylinder_final.wrl"
// 		Final output of the Segmentation Algorithm (Clusters which are predicted as Trees by the algo) is generated into "generated_wrl/med_cylinder/enclose/2_ac/tree_2_ac_final.wrl"
// 		"generated_wrl/med_cylinder/enclose/2_ac/tree_2_ac.wrl" is same as output of 2-(a) & can be ignored.

#include <cmath>
#include <string>
#include <bits/stdc++.h>
#include <regex> 
#include <iomanip>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>

using namespace std;

// ******************************************************************************************************************
// If clusters formed by grouping 3D points(output of 1st part of algo) is available, don't form clusters again. Only filter the clusters(2nd part of algo)
int do_clustering = 0;
// ******************************************************************************************************************

vector<vector<float>> cylinder;
vector<float> final_med_cylinder_x;
vector<float> final_med_cylinder_y;
vector<float> final_med_cylinder_r;
vector<string> viz_coordinate;
vector<string> viz_color;
vector<string> viz_color_final;
vector<string> viz_coordinate_final;
ofstream log_write;
string clusters_path;

// ************************************ Various thresholds used *****************************************************
// Should tune these values & see if better results can be obtained
int combinations_threshold = 1000000; //hyper-parameter
float collinearity_threshold = 0.5;  //hyper-parameter
float proximity_threshold = 0.7;  //0.5 //hyper-parameter
float tree_radius_thresh = 8.0;  //hyper-parameter
float cluster_enclosing_threshold = 1.0; //hyper-parameter
int cluster_size_low_threshold = 50; //hyper-parameter
int cluster_size_high_threshold = 2400; //hyper-parameter
// ******************************************************************************************************************


void remove_ground_pts_from_cluster(vector<vector<float>>& curr_cluster, float max_z, float min_z) {
	//if Number of pts having their z-coordinate in the bottom 20% of the range (min_z, max_z) is more than 35% of all the points in the cluster, then remove all those pts (whose z-coordinates lie in the bottom 20% of the range (min_z, max_z))
	//we hope to remove all the ground points which are surrounding the base of the trunk(in worst case, a few points at the base of the trunk too).
	//These 20% & 35% are tunable parameters, saved in the variables z_coord_threshold & num_points_threshold respectively.
	
	float z_coord_threshold = 0.2; // hyper-parameter
	float num_points_threshold = 0.35; // hyper-parameter
	int count = 0;
	for (int ml=0; ml<curr_cluster.size(); ml++){
		if ((curr_cluster[ml][2] >= min_z) && (curr_cluster[ml][2] <= (min_z+z_coord_threshold*(max_z-min_z)))) {
			count++;
		}
	}
	if (count >= (curr_cluster.size()*num_points_threshold)) {
		for (int rl=0; rl<curr_cluster.size(); rl++){
			if ((curr_cluster[rl][2] >= min_z) && (curr_cluster[rl][2] <= min_z+z_coord_threshold*(max_z-min_z))) {
				curr_cluster.erase(curr_cluster.begin()+rl);
				rl--;
			}
		}
	}
	return;
}

tuple<float,float> cluster_z_range(vector<vector<float>>& curr_cluster) {
	float max_z = -1000;
	float min_z = 1000;
	for (int ds=0; ds< curr_cluster.size(); ds++) {
		if (curr_cluster[ds][2]>max_z){max_z = curr_cluster[ds][2];}
		if (curr_cluster[ds][2]<min_z){min_z = curr_cluster[ds][2];}
	}
	return make_tuple(max_z,min_z);
}


float distance_bw_points (vector<float>& point1, vector<float>& point2) {
	return sqrt(pow((point1[0]-point2[0]),2)+pow((point1[1]-point2[1]),2)+pow((point1[2]-point2[2]),2));
}

float determinantOfMatrix(vector<vector<float>> &mat) 
{ 
	float ans; 
	ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) 
		  - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) 
		  + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]); 
	return ans; 
} 
  
void findSolution(vector<vector<float>> &coeff, int flag2) 
{ 

	vector<vector<float>> d;
	d.push_back({ coeff[0][0], coeff[0][1], coeff[0][2] });
	d.push_back({ coeff[1][0], coeff[1][1], coeff[1][2] });
	d.push_back({ coeff[2][0], coeff[2][1], coeff[2][2] });

	vector<vector<float>> d1;
	d1.push_back({ coeff[0][3], coeff[0][1], coeff[0][2] });
	d1.push_back({ coeff[1][3], coeff[1][1], coeff[1][2] });
	d1.push_back({ coeff[2][3], coeff[2][1], coeff[2][2] });

	vector<vector<float>> d2;
	d2.push_back({ coeff[0][0], coeff[0][3], coeff[0][2] });
	d2.push_back({ coeff[1][0], coeff[1][3], coeff[1][2] });
	d2.push_back({ coeff[2][0], coeff[2][3], coeff[2][2] });

	vector<vector<float>> d3;
	d3.push_back({ coeff[0][0], coeff[0][1], coeff[0][3] });
	d3.push_back({ coeff[1][0], coeff[1][1], coeff[1][3] });
	d3.push_back({ coeff[2][0], coeff[2][1], coeff[2][3] });
  

	float D = determinantOfMatrix(d); 
	float D1 = determinantOfMatrix(d1); 
	float D2 = determinantOfMatrix(d2); 
	float D3 = determinantOfMatrix(d3); 
	
  
	if (D != 0) { 
		float s = D1 / D; 
		float t = D2 / D; 
		float u = D3 / D;
		float temp31 = pow(s,2)+pow(t,2)-u;
		if (temp31 < 0) {return;}
		float r = sqrt(temp31);
		if (!isnan(r)){
				cylinder.push_back({s,t,r});
		} else { // If 3 points are collinear, then the cylinder radius becomes infinite. So, append a large number as radius.
			cylinder.push_back({s,t,100000.0});
		}
		
	}
}

void create_combinations(vector<vector<vector<float>>>& combinations, vector<vector<float>>& reduced_cluster1, long int num_elements) {
	long int actual_combinations = (num_elements*(num_elements-1)*(num_elements-2));
	// Instead of taking all possible combinations of 3 points from the cluster, take 1000000 random combinations & use them to find the median cylinder.
	// This reduces the computational intensity & also found that, this wouldn't change the median_radius much.
	int max_combinations;
	if (actual_combinations > combinations_threshold) {
		max_combinations = combinations_threshold;
	} else {
		max_combinations = actual_combinations;
	}
	// cout << actual_combinations << "|" << num_elements << "|"<< max_combinations<< endl; // change
	// return; // change
	std::random_device rd; // below snippet of code ensures different seed values across executions
	std::mt19937::result_type seed1 = rd() ^ ((std::mt19937::result_type) std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() + (std::mt19937::result_type) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
	std::mt19937 gen1(seed1);
	std::mt19937::result_type seed2 = rd() ^ ((std::mt19937::result_type) std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() + (std::mt19937::result_type) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
	std::mt19937 gen2(seed2);
	std::mt19937::result_type seed3 = rd() ^ ((std::mt19937::result_type) std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() + (std::mt19937::result_type) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
	std::mt19937 gen3(seed3);
	std::uniform_int_distribution<unsigned> distrib(0, num_elements-1);
	int index1, index2, index3;
	vector<string> combination_indices = {};
	vector<int> temp_vec;
	vector<string>::iterator itr;

	while (combinations.size() != max_combinations) {
		index1 = distrib(gen1);
		index2 = distrib(gen2);
		while (index2 == index1) {index2 = distrib(gen2);}
		index3 = distrib(gen3);
		while ((index3 == index2) || (index3 == index1)) {index3 = distrib(gen3);}
		temp_vec = {index1, index2, index3};
		combinations.push_back({reduced_cluster1[index1], reduced_cluster1[index2], reduced_cluster1[index3]});
	}
}

tuple<float,float,float> find_median_radius (vector<vector<float>>& cluster, int visualize, int curr_cluster_num, string color_line) {
	// To get an estimate of the cluster's curvature, find the median of radii of circles passing through XY-plane projections of all possible 3-point combinations from the cluster.
	// This is better than finding a circle which encloses the cluster, because of robustness to outliers & being influenced by most points.
	// Finding circles which pass through all possible 3-point combinations is computationally very expensive.
	// So, the following 2 steps are taken to reduce the compytational intensity:
	// 1) After projecting all points onto XY-plane, consider only unique points. (But, found that, this didn't reduce the intensity to a satisfactory level. Hence, the next step.)
	// 2) Take only 10^6 random combinations instead of all possible cobinatins.
	// This function also generates info needed to generate wrl file to: visualize each cluster & the median cylinder corresponding to it.
	// Finally, one wrl file for all clusters is generated.(This is done in "Visualize_med_cylinder" function.)
	float max_z = -1000;
	float min_z = 1000;
	vector<vector<float>> aug_mat;//[3][4] = {
	vector<vector<float>> combo;
	vector<vector<float>>().swap(cylinder);
	if (cylinder.size() != 0) {
		return make_tuple(-121.0, -121.0, -121.0);
	}
	int bl, cl, pl;

	cout << "***** curr_cluster_num: " << curr_cluster_num << endl;
	log_write << "***** curr_cluster_num: "+to_string(curr_cluster_num)+"\n";
	cout << "Un-reduced cluster size: " << cluster.size() << endl;
	log_write << "Un-reduced cluster size: "+to_string(cluster.size())+"\n";

	// Reducing cluster to have only points with unique X,Y coordinates.
	// This reduces the computational intensity & also found that, this wouldn't change the median_radius much.
	vector<float> reduced_cluster_x;
	vector<float> reduced_cluster_y;
	// vector<float> reduced_cluster_z;
	vector<vector<float>> reduced_cluster;
	for (pl=0; pl<cluster.size(); pl++ ){
		if ((find(reduced_cluster_x.begin(),reduced_cluster_x.end(),cluster[pl][0]) == reduced_cluster_x.end()) || (find(reduced_cluster_y.begin(),reduced_cluster_y.end(),cluster[pl][1]) == reduced_cluster_y.end())) {
			reduced_cluster_x.push_back(cluster[pl][0]);
			reduced_cluster_y.push_back(cluster[pl][1]);
			// reduced_cluster_z.push_back(cluster[pl][2]);
			reduced_cluster.push_back({cluster[pl][0], cluster[pl][1], cluster[pl][2]});
		}
		if (cluster[pl][2]>max_z){max_z = cluster[pl][2];}
		if (cluster[pl][2]<min_z){min_z = cluster[pl][2];}
	}
	// cout << "Reduced cluster size: " << reduced_cluster.size() << endl;
	//############ END ###################
	
	vector<vector<vector<float>>> random_combinations = {};
	create_combinations(random_combinations, reduced_cluster, reduced_cluster.size());
	// return make_tuple(-121.0, -121.0, -121.0); // change
	for (int ya = 0; ya < random_combinations.size(); ya++) {
		vector<vector<float>>().swap(combo);
		combo.push_back(random_combinations[ya][0]);
		combo.push_back(random_combinations[ya][1]);
		combo.push_back(random_combinations[ya][2]);
		
		vector<vector<float>>().swap(aug_mat);
		// ********* Find if there is a circle passing through the projections of the 3 selected points points onto XY-plane ******
		// ********* Use Cramer's rule to find solution of a system of linear equations.
		aug_mat.push_back({2*combo[0][0], 2*combo[0][1], -1, pow(combo[0][0],2)+pow(combo[0][1],2)});
		aug_mat.push_back({2*combo[1][0], 2*combo[1][1], -1, pow(combo[1][0],2)+pow(combo[1][1],2)});
		aug_mat.push_back({2*combo[2][0], 2*combo[2][1], -1, pow(combo[2][0],2)+pow(combo[2][1],2)});
		findSolution(aug_mat, 0);
		// ******************************************************************************************
	}

	vector<float> avg_cylinder_x; //cumulate all obtained cylinders & find the median
	vector<float> avg_cylinder_y;
	vector<float> avg_cylinder_r;
	for (int as=0; as<cylinder.size(); as++){
		avg_cylinder_x.push_back(cylinder[as][0]);
		avg_cylinder_y.push_back(cylinder[as][1]);
		avg_cylinder_r.push_back(cylinder[as][2]);
	}
	// Since we want to find the median, it is enough to sort the vector only till middle element
	std::nth_element(avg_cylinder_x.begin(), avg_cylinder_x.begin()+(avg_cylinder_x.size()/2)+1,avg_cylinder_x.end());
	std::nth_element(avg_cylinder_y.begin(), avg_cylinder_y.begin()+(avg_cylinder_y.size()/2)+1,avg_cylinder_y.end());
	std::nth_element(avg_cylinder_r.begin(), avg_cylinder_r.begin()+(avg_cylinder_r.size()/2)+1,avg_cylinder_r.end());
	
	float median_cylinder_x,median_cylinder_y,median_cylinder_r;
	// cout << "Number of cylinders in the reduced_cluster: " << avg_cylinder_x.size() << endl;
	if ((avg_cylinder_x.size()%2) == 0) {
		median_cylinder_x = (avg_cylinder_x[(avg_cylinder_x.size()/2)-1]+avg_cylinder_x[(avg_cylinder_x.size()/2)])/2.0;
	} else {
		median_cylinder_x = avg_cylinder_x[(avg_cylinder_x.size()/2)];
	}
	
	if ((avg_cylinder_y.size()%2) == 0) {
		median_cylinder_y = (avg_cylinder_y[(avg_cylinder_y.size()/2)-1]+avg_cylinder_y[(avg_cylinder_y.size()/2)])/2.0;
	} else {
		median_cylinder_y = avg_cylinder_y[(avg_cylinder_y.size()/2)];
	}
	
	if ((avg_cylinder_r.size()%2) == 0) {
		median_cylinder_r = (avg_cylinder_r[(avg_cylinder_r.size()/2)-1]+avg_cylinder_r[(avg_cylinder_r.size()/2)])/2.0;
	} else {
		median_cylinder_r = avg_cylinder_r[(avg_cylinder_r.size()/2)];
	}

	cout << "Radius of median cylinder: " << median_cylinder_r << endl;
	log_write << "Radius of median cylinder: "+to_string(median_cylinder_r)+"\n";
	// log_file << "Radius of median "<< type <<" cylinder: "+to_string(median_cylinder_r)+"\n";
	if (visualize) {
		string var1 = "      ";
		string var5 = ",\n";
		string var2, var3, var4;
		for (int ap=0; ap<cluster.size(); ap++) { // store data about the current cluster to write to wrl file in 'Visualize_med_cylinder' function.
			var2 = to_string(cluster[ap][0]);
			var3 = to_string(cluster[ap][1]);
			var4 = to_string(cluster[ap][2]);
			viz_color.push_back(color_line);
			viz_coordinate.push_back(var1+var2+' '+var3+' '+var4+var5);
		}

		string temp_x, temp_y;
		int count21=0;
		for (int aq=0; aq<360; aq+=20){ //In the wrl file,include the median cylinder corresponding to the cluster along with the cluster
			for (float qa=min_z; qa<max_z; qa+=(max_z-min_z)/10.0 ) {
				viz_color.push_back(color_line);
				temp_x = to_string(median_cylinder_x+median_cylinder_r*cos(aq*3.14159/180));
				temp_y = to_string(median_cylinder_y+median_cylinder_r*sin(aq*3.14159/180));
				viz_coordinate.push_back(var1+temp_x+' '+temp_y+' '+to_string(qa)+var5);
			}
		}
	}
	// time(&end_med_rad);
	// double time_taken = double(end_med_rad - start_med_rad);
	// cout << "Time taken for cluster-"<< curr_cluster_num <<  " is : " << time_taken << endl;
	return make_tuple(median_cylinder_x,median_cylinder_y,median_cylinder_r);
}

void generate_final_data (vector<vector<float>>& cluster, int visualize, int curr_cluster_num, string color_line) {
	// Generates the info needed to generate wrl file for the final segmentation of trees.
	// The wrl file is actually generated in "Visualize_med_cylinder" function.
	float max_z = -1000;
	float min_z = 1000;
	cout << "***** curr_cluster_num: " << curr_cluster_num << endl;
	log_write << "***** curr_cluster_num: "+to_string(curr_cluster_num)+"\n";
	vector<vector<float>> reduced_cluster;
	for (int pl=0; pl<cluster.size(); pl++ ){
		if (cluster[pl][2]>max_z){max_z = cluster[pl][2];}
		if (cluster[pl][2]<min_z){min_z = cluster[pl][2];}
	}

	if (visualize) {
		float median_cylinder_x, median_cylinder_y, median_cylinder_r;
		median_cylinder_x = final_med_cylinder_x[curr_cluster_num];
		median_cylinder_y = final_med_cylinder_y[curr_cluster_num];
		median_cylinder_r = final_med_cylinder_r[curr_cluster_num];
		string var1 = "      ";
		string var5 = ",\n";
		string var2, var3, var4;
		for (int ap=0; ap<cluster.size(); ap++) {
			var2 = to_string(cluster[ap][0]);
			var3 = to_string(cluster[ap][1]);
			var4 = to_string(cluster[ap][2]);
			viz_color_final.push_back(color_line);
			viz_coordinate_final.push_back(var1+var2+' '+var3+' '+var4+var5);
		}
	}
}

void Visualize_med_cylinder (string env, string name, int flag2) {
	//If flag2 ==0, Generate wrl to visualize clusters (with relevant radius(radius < threshold) of the corresponding median cylinder).
	//If flag2 ==1, Generate the final wrl file containing segmented trees
	ofstream myfile;
	string temp_file1 = "generated_wrl/med_cylinder/enclose/";
	string temp_file2 = "/"+name+"_"+env;
	string temp_file3;
	if (flag2==0){temp_file3 = ".wrl";} 
	if (flag2==1){temp_file3 = "_final.wrl";}
	myfile.open (temp_file1+env+temp_file2+temp_file3);

	myfile << "#VRML V1.0 ascii\n";
	myfile << "\n";
	myfile << "Separator { \n";
	myfile << "  MaterialBinding { \n";
	myfile << "        value PER_VERTEX_INDEXED \n";
	myfile << "  }\n";
	myfile << "\n";
	myfile << "  Material { \n";
	myfile << "    diffuseColor [ \n";
	
	if (flag2==0){
		for (int yak=0; yak<viz_color.size(); yak++ ){
			myfile <<viz_color[yak];
		}
	}
	if (flag2==1){
		for (int yak=0; yak<viz_color_final.size(); yak++ ){
			myfile <<viz_color_final[yak];
		}
	}

	myfile << "       0 0 0 ]\n"; // adding a black point at origin, just to make coding easy(This wouldn't affect the clusters as the point is added only while visualization).
	myfile << "  } \n";
	myfile << "\n";
	myfile << "  Coordinate3 { \n";
	myfile << "    point [ \n";

	if (flag2==0){
		for (int yam=0; yam<viz_coordinate.size(); yam++ ){
			myfile <<viz_coordinate[yam];
		}
	}
	if (flag2==1){
		for (int yam=0; yam<viz_coordinate_final.size(); yam++ ){
			myfile <<viz_coordinate_final[yam];
		}
	}
	
	myfile << "       0 0 0 ]\n";// adding a black point at origin, just to make coding easy(This wouldn't affect the clusters as the point is added only while visualization).
	myfile << "}\n";
	myfile << "\n";
	myfile << "  PointSet { \n";
	myfile << "    startIndex 0 \n";
	string var7 = "    numPoints ";
	string var8 = "\n";
	string num_points;
	if (flag2==0){num_points = to_string(viz_color.size()+1);}
	if (flag2==1){num_points = to_string(viz_color_final.size()+1);}
	myfile << var7+num_points+var8;
	myfile << "  } \n";
	myfile << "} \n";

	myfile.close();
	// cout << "Generated wrl file for clusters" << endl;
}

void Visualize (vector<vector<float>>& vec_i, string name, string env) {
	// Function to visualize the clusters formed by grouping 3D points (in 1st step of algorithm.)
	ofstream myfile;
	string temp_file1 = "generated_wrl/"+env+"/cluster_";
	string temp_file2 = name;//"tree";//_oakland_part2_ak";
	string temp_file3 = ".wrl";
	myfile.open (temp_file1+temp_file2+temp_file3);

	myfile << "#VRML V1.0 ascii\n";
	myfile << "\n";
	myfile << "Separator { \n";
	myfile << "  MaterialBinding { \n";
	myfile << "        value PER_VERTEX_INDEXED \n";
	myfile << "  }\n";
	myfile << "\n";
	myfile << "  Material { \n";
	myfile << "    diffuseColor [ \n";
	
	for (int yak=0; yak<vec_i.size(); yak++ ){
		myfile << "      "+to_string(0)+' '+to_string(0)+' '+to_string(1)+",\n";
	}

	myfile << "       0 0 0 ]\n";// adding a point at origin, just to make coding easy(This wouldn't affect the clusters as the point is added only while visualization).
	myfile << "  } \n";
	myfile << "\n";
	myfile << "  Coordinate3 { \n";
	myfile << "    point [ \n";

	
	for (int yam=0; yam<vec_i.size(); yam++ ){
		myfile <<"      "+to_string(vec_i[yam][0])+' '+to_string(vec_i[yam][1])+' '+to_string(vec_i[yam][2])+",\n";
	}

	myfile << "       0 0 0 ]\n";// adding a point at origin, just to make coding easy(This wouldn't affect the clusters as the point is added only while visualization).
	myfile << "}\n";
	myfile << "\n";
	myfile << "  PointSet { \n";
	myfile << "    startIndex 0 \n";
	string var7 = "    numPoints ";
	string var8 = "\n";
	string num_points = to_string(vec_i.size()+1);
	myfile << var7+num_points+var8; 
	myfile << "  } \n";
	myfile << "} \n";

	myfile.close();
	// cout << "Generated wrl file for clusters" << endl;
}

void list_dir(const char* path, vector<string>& vec2) {//str.c_str()
	// read all file nams of the wrl files corresponding to all steps (of cluster growth) for each cluster & select only the final step for each cluster 
	struct dirent *entry;
	vector<string> vec1;
	DIR *dir = opendir(path);

	if (dir == NULL) {
		return;
	}
	while ((entry = readdir(dir)) != NULL) {
		vec1.push_back(entry->d_name);
	}
	closedir(dir);
	sort(vec1.begin(), vec1.end());//, greater<string>());

	vector<vector<int>> clusters_and_steps(1000, vector<int>());
	int curr_cluster_num;//, prev_cluster_num=0;
	string x;
	for (int i=0; i<vec1.size(); i++) {
		x = vec1[i];
		
		regex re("cluster_(\\d*)_step(\\d*)_after_gnd_removal.wrl");
		for (sregex_iterator it = sregex_iterator(x.begin(), x.end(), re); it != sregex_iterator(); it++) {
			smatch match;
			match = *it;
			curr_cluster_num = stoi(match.str(1));
			clusters_and_steps[curr_cluster_num].push_back(stoi(match.str(2)));
		}
	}

	for (int i=1; i<clusters_and_steps.size(); i++) {
		if (clusters_and_steps[i].size() == 0) {
			break;
		}
		sort(clusters_and_steps[i].begin(), clusters_and_steps[i].end());
		vec2.push_back(clusters_path+"cluster_"+to_string(i)+"_step"+to_string(clusters_and_steps[i][clusters_and_steps[i].size()-1])+"_after_gnd_removal.wrl");
	}
}

float distance_bw_points_projection (vector<float>& point1, vector<float>& point2) {
	return sqrt(pow((point1[0]-point2[0]),2)+pow((point1[1]-point2[1]),2));
}

int check_if_cluster_resides_inside_median_cylinder (vector<vector<float>>& cluster, int ul) {
	int total_points = cluster.size();
	// cluster_enclosing_threshold = 0.98;
	int enclosed_points = 0;
	vector<float> med_circle_center = {final_med_cylinder_x[ul], final_med_cylinder_y[ul]};
	for (int f1 = 0; f1<cluster.size(); f1++) {
		if (distance_bw_points_projection(med_circle_center, cluster[f1]) <= final_med_cylinder_r[ul]) {
			enclosed_points++;
		}
	}
	if (enclosed_points >= (total_points*cluster_enclosing_threshold)) {
		return 1;
	} else {return 0;}
}

int main (int argc, char** argv){
	string environment = argv[1];
	int pts_in_env = stoi(argv[2]);
	clusters_path = "generated_wrl/"+environment+"/z_range_0.2_0.35_proxi_0.7/";
	log_write.open ("log/enclose/log_"+environment+".txt");
	
	time_t start_cluster, end_cluster; 
	// ############################# Beginning of Read WRL File #############################
	vector<vector<float>> coordinate;
	string file_name = "../wrl/oakland_part"+environment+".wrl";
	// do_clustering = 0;
	// cout << "do_clustering: "<< do_clustering << endl;
	if (do_clustering != 0) {
		ifstream inFile;
		string x;

		cout << "environment: " << environment << endl;
		inFile.open(file_name);

		if (!inFile) {
			cout << "Unable to open file" << endl;
				exit(1); // terminate with error
		}

		int flag = 0;
		int count = 0;
		while (getline(inFile, x)) {
			count ++;
			if (count < pts_in_env) {continue;}
			if (flag == 0) {
				if (regex_match (x,regex (".*Coordinate3.*"))) {
					flag = 1;
				}
			}
			if (flag == 1) {
			regex re(".*\\s(.*\\d.*)\\s(.*\\d.*)\\s(.*\\d.*)((\\,)|(\\s)).*");
			vector<float> vec1;
			for (sregex_iterator it = sregex_iterator(x.begin(), x.end(), re); it != sregex_iterator(); it++) {
				smatch match;
				match = *it;
				vec1.push_back(stof(match.str(1)));
				vec1.push_back(stof(match.str(2)));
				vec1.push_back(stof(match.str(3)));
			}
			if (vec1.size() != 0) {coordinate.push_back(vec1);}
			}
		}

		cout << "Size of wrl coordinate array: " << coordinate.size() << endl;
		if (coordinate.size() < 40826) {//smallest wrl file in the datste used had 40826 points
			cout << "Error in Reading WRL File. Exiting.";
			return 0;
		}
		inFile.close();
	}
	//######################################### End of Read WRL File #############################

	//############################# Start Grouping the 3D points into Clusters(1st part of algo) #########################################
	vector<vector<vector<float>>> clusters; // variable to store each cluster separately
	vector<string> all_files;
	if (do_clustering == 1) { // if clustering needs to be done (not just filtering of already available clusters), generate the clusters
		time(&start_cluster);
		int count1 = 0;
		string junk;
		int flag1, ngbrs_to_curr_pt, total_points = 0;
		float cluster_z_range_threshold=(0.4)*(2*proximity_threshold);//hyper-parameter
		float range,max_z,min_z;
		int curr_set_idx, next_set_idx, curr_count = 0, temp_idx, num_steps=0;
		while (coordinate.size() != 0) {
			cout << "********************* Starting cluster-" << count1+1 << endl;
			clusters.push_back({coordinate[0]});
			coordinate.erase(coordinate.begin());
			curr_set_idx = 0;//each set corresponds to one iteration of adding points to cluster
			next_set_idx = 1;
			num_steps=0;
			curr_count=0;
			for (int dl=0; dl<clusters[count1].size(); dl++){
				// cout << "3x";
				// cout << dl << " | "<< curr_set_idx << " | " << next_set_idx << " | " <<curr_count<< endl;
				if (dl>=next_set_idx) {
					// temp_idx = curr_set_idx;
					curr_set_idx = next_set_idx;
					next_set_idx = curr_set_idx+curr_count;
					curr_count = 0;
					// cout << dl << " | "<< curr_set_idx << " | " << next_set_idx << " | " <<curr_count<< "******" << endl;
				}
				flag1 = 0;
				for (int rl=0; rl<coordinate.size(); rl++){
					if (distance_bw_points(clusters[count1][dl],coordinate[rl]) <= proximity_threshold) {
						clusters[count1].push_back(coordinate[rl]);
						coordinate.erase(coordinate.begin()+rl); // removing elements from the vector `coordinate` to reduce computational intensity(becz, there is no need to check proximity of one cluster point to another cluster point)
						rl--;
						if ((dl>= curr_set_idx) && (dl< next_set_idx)) {// just keeping the if-condition although its redundant.
							curr_count++;
						}
					}
				}
				tie(max_z, min_z) = cluster_z_range(clusters[count1]);
				range = max_z-min_z;
				if (range < cluster_z_range_threshold) { 
					// if there is no sufficient variation in z-coordinates of the points in cluster, it means that the current cluster is a part of a horizontal surface (like road) & can't be a part of tree.
					//  & hence discard the current cluster.
					clusters.erase(clusters.end()-1);
					count1--;
					flag1=1;
					// cout << "##### One cluster skipped.";
					break;
				}
				if (dl==(next_set_idx-1)) { // if one step of growth is completed
					// Allow the cluster to grow in all directions for one step. & then check if the cluster has any points corresponding to ground.
					// If the cluster has points corresponding to both ground & tree, remove the ground points(so that cluster won't grow in ground direction in next step) & continue growing only in tree direction.
					num_steps++;
					// cout << "Enter if checking is done.";
					// cin >> junk;
					Visualize(clusters[count1], to_string(count1+1)+"_step"+to_string(num_steps)+"_before_gnd_removal", environment);
					// cout << "Cluster-" << count1+1 << " size after step-"<<num_steps<<": " << clusters[count1].size() << endl;
					cout << clusters[count1].size() << " | " ;
					remove_ground_pts_from_cluster(clusters[count1], max_z, min_z);
					// cout << "Cluster-" << count1+1 << " size after step-"<<num_steps<<": " << clusters[count1].size() << endl;
					Visualize(clusters[count1], to_string(count1+1)+"_step"+to_string(num_steps)+"_after_gnd_removal", environment);
					cout << clusters[count1].size() << " || ";
					// cout << "One step of growing is done." << endl;
					// cout << "1x";
				}
				// cout << "2x";
			}
			if (flag1 == 0) {
				total_points += clusters[count1].size();
				cout << "Number of points in current cluster = " << clusters[count1].size() << endl;
			}
			// cout << "4x";
			count1++;
		}
		cout << "Total Number of clusters in '" << file_name << "': " << clusters.size() << endl;

		// After accumulating 200 points, check variability of z-coordinates in cluster
		// Don't abruptly stop expanding the cluster once its size reaches 200. Allow it to continue the current growth step & then stop to check whether there is variability in z-coordinates of the points in cluster
		// after every step (of adding 200 points), remove points whose z-coordinates are almost similar & continue expanding same cluster.
		// First check height(variability in z-coordinates) & then check if the obtained tall cluster is circular in XY-Plane.


		time(&end_cluster);
		double time_taken = double(end_cluster - start_cluster);
		cout << "Time taken for Clustering with proximity_threshold: "<< proximity_threshold <<  " is : " << time_taken << endl;
	} else {
		// If wrl files corresponding to 1st step of algo are already available, read them & proceed to 2nd step of algo.
		vector<vector<float>> cluster_coordinate;
		list_dir(clusters_path.c_str(), all_files);
		int flag123;

		for (int i=0; i<all_files.size(); i++) {
			ifstream inFile;
			string y;
			inFile.open(all_files[i]);
			if (!inFile) {
				cout << "Unable to open file" << endl;
				log_write << "Unable to open file\n";
					exit(1); // terminate with error
			}
			flag123 = 0;
			vector<vector<float>>().swap(cluster_coordinate);
			while (getline(inFile, y)) {

				if (regex_match (y,regex (".*Coordinate3.*"))) {flag123 = 1;}
				if (flag123 == 0) {continue;}
				
				if (flag123 == 1) {
				regex re(".*\\s(.*\\d.*)\\s(.*\\d.*)\\s(.*\\d.*)((\\,)|(\\s)).*");
				vector<float> vec12;
				for (sregex_iterator it = sregex_iterator(y.begin(), y.end(), re); it != sregex_iterator(); it++) {
					smatch match;
					match = *it;
					vec12.push_back(stof(match.str(1)));
					vec12.push_back(stof(match.str(2)));
					vec12.push_back(stof(match.str(3)));
				}
				if (vec12.size() != 0) {cluster_coordinate.push_back(vec12);}
				}
				if (cluster_coordinate.size() > cluster_size_high_threshold+2) {break;} //(To speeden the execution), if a cluster size is more than `cluster_size_high_threshold`, don't even read the entire wrl file corresponding to that cluster.
			}
			clusters.push_back(cluster_coordinate);
		}
		cout << "********* number of clusters: " << clusters.size() << endl;
		log_write << "********* number of clusters: "+to_string(clusters.size())+"\n";
	}
	//############################# END Grouping the 3D points into Clusters(End of 1st part of algo) #########################################
	
	// ***************************** 2nd step of algorithm ***************************** 
	// ######################### Start filtering the clusters(2nd part of algo) #########################################
	
	for (int ag=0; ag<clusters.size(); ag++){ // (1st stage of filtering) Remove clusters with irrelevant sizes
		if ((clusters[ag].size() < cluster_size_low_threshold) || (clusters[ag].size() > cluster_size_high_threshold)) {
			clusters.erase(clusters.begin()+ag);
			all_files.erase(all_files.begin()+ag);
			ag--;
		} 
		else {
			// valid_cluster_num++;
			// cout << all_files[ag] << endl;
			// cout << "size of cluster-" << ag+1 << ": " << clusters[ag].size() <<endl;
		}
	}
	cout << "********* number of clusters of relevant sizes: " << clusters.size() << endl;
	log_write << "********* number of clusters of relevant sizes: "+to_string(clusters.size())+"\n";

	float median_cluster_radius, med_cyl_x, med_cyl_y;
	int valid_cluster_num = -1;
	string color_line;
	int ai=0;
	time_t start_cls_filter, end_cls_filter;
	time(&start_cls_filter);
	for (int al=0; al<clusters.size(); al++, ai++) {
		valid_cluster_num++;
		if ((al%6)==0) {valid_cluster_num=0;}
		if (valid_cluster_num==0) {color_line = "      0 0 1,\n";}
	    if (valid_cluster_num==1) {color_line = "      0 1 0,\n";}
	    if (valid_cluster_num==2) {color_line = "      1 0 0,\n";}
	    if (valid_cluster_num==3) {color_line = "      1 1 0,\n";}
	    if (valid_cluster_num==4) {color_line = "      0 1 1,\n";}
	    if (valid_cluster_num==5) {color_line = "      1 0 1,\n";}

	    cout << endl << "***** Current cluster: " << all_files[ai] << endl;
	    log_write << "\n***** Current cluster: "+all_files[ai]+"\n";
		tie(med_cyl_x, med_cyl_y, median_cluster_radius) = find_median_radius(clusters[al], 1, ai, color_line);
		if (median_cluster_radius == -121.0) {return 0;}
		Visualize_med_cylinder(environment, "tree", 0);

		// Start 2nd stage of filtering
		if (median_cluster_radius > tree_radius_thresh){
			clusters.erase(clusters.begin()+al);
			al--;
		} else {
			final_med_cylinder_x.push_back(med_cyl_x);
			final_med_cylinder_y.push_back(med_cyl_y);
			final_med_cylinder_r.push_back(median_cluster_radius);
		}
		// End 2nd stage of filtering
	}
	// Start 3rd stage of filtering
	for (int ul=0; ul<clusters.size(); ul++){
		if (check_if_cluster_resides_inside_median_cylinder(clusters[ul], ul)) {
			clusters.erase(clusters.begin()+ul);
			ul--;
		}
	}
	// End 3rd stage of filtering

	time(&end_cls_filter);
	cout << "Time taken for filtering the clusters is : " << double(end_cls_filter-start_cls_filter) << endl;
	log_write << "Time taken for filtering the clusters is : " + to_string(float(end_cls_filter-start_cls_filter))+"\n";

	// Visualize the final output of the algo
	valid_cluster_num = -1;
	for (int al=0; al<clusters.size(); al++) {
		valid_cluster_num++;
		if ((al%6)==0) {valid_cluster_num=0;}
		if (valid_cluster_num==0) {color_line = "      0 0 1,\n";}
	    if (valid_cluster_num==1) {color_line = "      0 1 0,\n";}
	    if (valid_cluster_num==2) {color_line = "      1 0 0,\n";}
	    if (valid_cluster_num==3) {color_line = "      1 1 0,\n";}
	    if (valid_cluster_num==4) {color_line = "      0 1 1,\n";}
	    if (valid_cluster_num==5) {color_line = "      1 0 1,\n";}
		generate_final_data(clusters[al], 1, al, color_line);
		Visualize_med_cylinder(environment, "tree", 1);
	}
	// ######################### END filtering the clusters #########################################
	log_write.close();
	return 0;
}
