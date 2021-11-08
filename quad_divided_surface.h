#pragma once

#include <array> 
#include "quad_tree.h"

#include "eigen/core"

// TODO(huaiyuan): 
// Determine the usage of the brother functions by 
// 1, Use  chuizhi index range to  build a sharp edge first. We skip polygon.

class QuadDividedCubicSurface {
public:
	// Vertex id definition:
	//  <--mas_widt-->  
	//  01__________11    02____12____22
	//  |            |    |      |     |
	//  |            |    01____11____21
	//  |            |    |      |     |
	//  00__________10    00____10____20
	//	   (level 0)         (level 1)
	//
	// QuadDividedCubicSurface takes samples at level-ed vertices from user.
	// to build up a piecewise bi-cubic surfaces.
	// It makes sure that the value at vertices are equaling with given samples,
	// (with those at bigger level overriding smaller ones).
	// The function is 2nd order continuos within the master box.
	QuadDividedCubicSurface(int quad_tree_depth , double master_box_width = 1.):
		sample_surfaces_(quad_tree_depth),
		master_box_width_(master_box_width){
		CHECK_GT(master_box_width, 0.0);
		CHECK_GT(quad_tree_depth, 0);

		sample_surface_cache_.reserve(quad_tree_depth);
	}
 
	// Pre-requisite:
	// IsPopulated().
	double operator()(double x, double y) const {
		DCHECK(IsPopulated());

		const SampleSurface* sample_surface = sample_surfaces_.GetElementByPosition(x, y, master_box_width_);
		if (sample_surface == nullptr) {
			return 0;
		}
		else {
			return (sample_surface->surface).GetValue(x, y);
		}
	}

	// Pre-requisite:
	// IsPopulated().
	double GetValueAndDerivative(double x, double y, double* df_dx, double* df_dy, double* d2f_dx2, double* d2f_dx_dy, double* d2f_dy2) const {
		DCHECK(IsPopulated());
		DCHECK(df_dx != nullptr);
		DCHECK(df_dy != nullptr);
		DCHECK(d2f_dx2 != nullptr);
		DCHECK(d2f_dx_dy != nullptr);
		DCHECK(d2f_dy2 != nullptr);
		const SampleSurface* sample_surface = sample_surfaces_.GetElementByPosition(x, y, master_box_width_);
		if (sample_surface == nullptr) {
			*df_dx = 0;
			*df_dy = 0;
			*d2f_dx2 = 0;
			*d2f_dx_dy = 0;
			*d2f_dy2 = 0;
			return 0.;
		}
		else {
			return sample_surface->surface.GetValueAndDerivative(x, y, df_dx,df_dy,d2f_dx2,d2f_dx_dy,d2f_dy2);
		}
	}

	
	// Reserve memory for the specified 
	// number of sample boxes.
	void reserve(int num_boxes) {
		sample_surfaces_.reserve(num_boxes);
	}


 
	void reset() {
		sample_surfaces_.clear();
		populated_ = false;
	}

	double master_box_width() const {
		return master_box_width_;
	}


	
	bool IsPopulated() const {
		return populated_;
	}


	void CreateStrip(double x_min, double y_min, double x_max, double y_max) {
		constexpr int kNum = 4;

		int level_ub = sample_surfaces_.depth();


		for (int level = 0; level < level_ub; ++level) {
			double width = get_width(level);
			YMonotoneGridSet left;
			// Boxes touching the ROI;

			double roi_right = std::min(x_min + width * 5, x_max);

			int xid_min = static_cast<int>(x_min / width);
			int xid_max = static_cast<int>(roi_right / width) + 1;

			int yid_min = static_cast<int>(y_min / width);
			int yid_max = static_cast<int>(y_max / width) + 1;


			if (xid_min % 2 != 0) xid_min -= 1;
			if (xid_max % 2 != 0) xid_max += 1;
			if (yid_min % 2 != 0) yid_min -= 1;
			if (yid_max % 2 != 0) yid_max += 1;

			 
			left.y_min_max.second = yid_max;
			left.y_min_max.first = yid_min;
					 
			left.x_min_max_list.resize(yid_max - yid_min + 1, std::make_pair(xid_min, xid_max));

			bool res  = AppendSamplesToLevel(left, level, [&x_min,&x_max, &y_min,&y_max](double x, double y, double* df_dx, double* df_dy, double* d2f_dx_dy) {
				if (x >= x_min && x<= x_max &&  y >= y_min && y<= y_max) {
					*df_dx = 1.0;
					*df_dy = 0;
					*d2f_dx_dy = 0;
					return x-x_min;
				}
				else {
					*df_dx = 0.0;
					*df_dy = 0;
					*d2f_dx_dy = 0;
					return 0.0;
				}
				});
			// Append the right;

			PopulateSamplesAtLevel(level);
		}

		populated_ = true;
		
	}



private:
	using RowVector4d = Eigen::Matrix<double, 1, 4>;
	using ColVector4d = Eigen::Matrix<double, 4, 1>;
	using Matrix4d = Eigen::Matrix<double, 4, 4>;
	// Need functionality:
	// Given GridSet of sample vertices of a level, get the 
	// 1, min required GridSet of boxes at this level, such that it has enough freedom to capture the samples.
	// 2, The required father level grid set, to support full quad tree.
	const std::vector<std::pair<int, int>> corner_dx_dy{ {0, 0, }, { 0,1 }, { 1,0 }, { 1,1 } };

	struct YMonotoneGridSet {
		std::pair<int,int> y_min_max;
		std::vector<std::pair<int, int>> x_min_max_list;

		// surounnding boxes: 

		//     o o o o o o 
		//   o o . . . . . o

		// Clamp by box.

		// Clamp by line.

		// From Convex Polygon.
	};


	struct VertexHelper {
		// vritual sample tag indicates the sample value shall be replaced when next time sampled.
		// Otherwise the values are added.
		bool is_virtual_sample = false;
	};

	struct Vertex {
		// NaN indicates the vertex is not sampled and should follow background.
		// During numerical action, NaN will be treated as zero.
		static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

		double f = kNaN;
		double df_dx = kNaN;
		double df_dy = kNaN;
		double d2f_dx_dy = kNaN;
		VertexHelper helper;

		inline bool IsNaN() const {
			return std::isnan(f);
		}

		static inline Vertex Zero() {
			return Vertex{ 0.,0.,0.,0. };
		}

		Vertex& operator+= (const Vertex& other) {
			if (IsNaN()) {
				f = other.f;
				df_dx = other.df_dx;
				df_dy = other.df_dy;
				d2f_dx_dy = other.d2f_dx_dy;
			} else if (!other.IsNaN()) {
				f += other.f;
				df_dx += other.df_dx;
				df_dy += other.df_dy;
				d2f_dx_dy += other.d2f_dx_dy;
			} // else no action.

			return *this;
		}
		
		Vertex& operator-= (const Vertex& other) {
			if (IsNaN()) {
				f = -other.f;
				df_dx = -other.df_dx;
				df_dy = -other.df_dy;
				d2f_dx_dy = -other.d2f_dx_dy;
			} else if (!other.IsNaN()) {
				f -= other.f;
				df_dx -= other.df_dx;
				df_dy -= other.df_dy;
				d2f_dx_dy -= other.d2f_dx_dy;
			} // else no action.

			return *this;
		}

		Vertex& operator*= (double gain) {
			if (!IsNaN()) {
				f *= gain;
				df_dx *= gain;
				df_dy *= gain;
				d2f_dx_dy *= gain;
			} // else no action.

			return *this;
		}

	};

	struct Sample {
		Vertex v00; 
		Vertex v01; 
		Vertex v10; 
		Vertex v11; 

		Sample& operator+= (const Sample& other) {
			v00 += other.v00;
			v01 += other.v01;
			v10 += other.v10;
			v11 += other.v11;
			return *this;
		}

		Sample& operator-= (const Sample& other) {
			v00 -= other.v00;
			v01 -= other.v01;
			v10 -= other.v10;
			v11 -= other.v11;
			return *this;
		}

		Sample& operator *=(double gain) {
			v00 *= gain;
			v01 *= gain;
			v10 *= gain;
			v11 *= gain;
			return *this;
		}


		inline bool HasNaN() const {
			return v00.IsNaN() ||
				v01.IsNaN() ||
				v10.IsNaN() ||
				v11.IsNaN();
		}


		Vertex& Corner(int dx, int dy) {
			// Assume dx \in {0,1}, dy \in {0,1}
			if (dx == 0) {
				if (dy == 0) {
					return v00;
				}
				else {
					return v01;
				}
			}
			else {
				if (dy == 0) {
					return v10;
				}
				else {
					return v11;
				}
			}
		}

	};

	struct Surface {
		double relative_x_min = 0.;
		double relative_y_min = 0.;
		Matrix4d a_mat;

		double GetValueAndDerivative(double x, double y, double* df_dx, double* df_dy, double* d2f_dx2, double* d2f_dx_dy, double* d2f_dy2) const {
			DCHECK(df_dx != nullptr);
			DCHECK(df_dy != nullptr);
			DCHECK(d2f_dx2 != nullptr);
			DCHECK(d2f_dx_dy != nullptr);
			DCHECK(d2f_dy2 != nullptr);

			x = x - relative_x_min;
			double x2 = x * x;
			double x3 = x * x2;

			y = y - relative_y_min;
			double y2 = y * y;
			double y3 = y * y2;

			RowVector4d x_polynomials;
			x_polynomials << 1, x, x2, x3;
			RowVector4d dx_polynomials;
			dx_polynomials << 0, 1, 2 * x, 3 * x2;
			RowVector4d d2x_polynomials;
			d2x_polynomials << 0, 0, 2, 6 * x;

			ColVector4d y_polynomials;
			y_polynomials << 1, y, y2, y3;
			ColVector4d dy_polynomials;
			dy_polynomials << 0, 1, 2 * y, 3 * y2;
			ColVector4d d2y_polynomials;
			d2y_polynomials << 0, 0, 2, 6 * y;

			ColVector4d a_mat_y = a_mat * y_polynomials;
			RowVector4d x_a_mat = x_polynomials * a_mat;

			*df_dx = dx_polynomials * a_mat_y;
			*d2f_dx2 = d2x_polynomials * a_mat_y;

			*df_dy = x_a_mat * dy_polynomials;
			*d2f_dy2 = x_a_mat * d2y_polynomials;

			*d2f_dx_dy = dx_polynomials * a_mat * dy_polynomials;

			return  x_polynomials * a_mat_y;
		}
		double GetValue(double x, double y) const {
			x = x - relative_x_min;
			double x2 = x * x;
			double x3 = x * x2;

			y = y - relative_y_min;
			double y2 = y * y;
			double y3 = y * y2;
			RowVector4d x_polynomials;
			x_polynomials << 1, x, x2, x3;

			ColVector4d y_polynomials;
			y_polynomials << 1, y, y2, y3;

			return  x_polynomials * a_mat * y_polynomials;
		}
	};
	
	struct SampleSurface {
		// The Unmerged sample.
		Sample sample;
		// Always the merged surface.
		Surface surface;

		// Update surface according to sample and background;
		void Populate(double x0, double y0, double width, const Sample& background) { 
			surface.relative_x_min = x0;
			surface.relative_y_min = y0;

			Sample abs_value = sample;
			abs_value += background;

			// Absolute value must be numerically deterministic.
			DCHECK(!abs_value.HasNaN());
			
			// Build surface matrix.
			double width_2 = width * width;
			double width_inv = 1. / width;

			double fr_matrix[16] = { abs_value.v00.f,  abs_value.v01.f,  abs_value.v00.df_dy * width,  abs_value.v01.df_dy * width, \
				abs_value.v10.f, abs_value.v11.f, abs_value.v10.df_dy * width, abs_value.v11.df_dy * width, \
				abs_value.v00.df_dx * width, abs_value.v01.df_dx * width, abs_value.v00.d2f_dx_dy * width_2, abs_value.v01.d2f_dx_dy * width_2, \
				abs_value.v10.df_dx * width, abs_value.v11.df_dx * width, abs_value.v10.d2f_dx_dy * width_2, abs_value.v11.d2f_dx_dy * width_2 };

			surface.a_mat <<
				fr_matrix[0],
				fr_matrix[2],
				(-3)* fr_matrix[0] + 3 * fr_matrix[1] - 2 * fr_matrix[2] - fr_matrix[3],
				2 * fr_matrix[0] - 2 * fr_matrix[1] + fr_matrix[2] + fr_matrix[3],
				fr_matrix[8],
				fr_matrix[10],
				(-3)* fr_matrix[8] + 3 * fr_matrix[9] - 2 * fr_matrix[10] - fr_matrix[11],
				2 * fr_matrix[8] - 2 * fr_matrix[9] + fr_matrix[10] + fr_matrix[11],
				(-3)* fr_matrix[0] + 3 * fr_matrix[4] - 2 * fr_matrix[8] - fr_matrix[12],
				(-3)* fr_matrix[2] + 3 * fr_matrix[6] - 2 * fr_matrix[10] - fr_matrix[14],
				9 * fr_matrix[0] - 9 * fr_matrix[1] + 6 * fr_matrix[2] + 3 * fr_matrix[3] - 9 * fr_matrix[4] + 9 * fr_matrix[5] - 6 * fr_matrix[6] - 3 * fr_matrix[7] + 6 * fr_matrix[8] - 6 * fr_matrix[9] + 4 * fr_matrix[10] + 2 * fr_matrix[11] + 3 * fr_matrix[12] - 3 * fr_matrix[13] + 2 * fr_matrix[14] + fr_matrix[15],
				(-6)* fr_matrix[0] + 6 * fr_matrix[1] - 3 * fr_matrix[2] - 3 * fr_matrix[3] + 6 * fr_matrix[4] - 6 * fr_matrix[5] + 3 * fr_matrix[6] + 3 * fr_matrix[7] - 4 * fr_matrix[8] + 4 * fr_matrix[9] - 2 * fr_matrix[10] - 2 * fr_matrix[11] - 2 * fr_matrix[12] + 2 * fr_matrix[13] - fr_matrix[14] - fr_matrix[15],
				2 * fr_matrix[0] - 2 * fr_matrix[4] + fr_matrix[8] + fr_matrix[12],
				2 * fr_matrix[2] - 2 * fr_matrix[6] + fr_matrix[10] + fr_matrix[14],
				(-6)* fr_matrix[0] + 6 * fr_matrix[1] - 4 * fr_matrix[2] - 2 * fr_matrix[3] + 6 * fr_matrix[4] - 6 * fr_matrix[5] + 4 * fr_matrix[6] + 2 * fr_matrix[7] - 3 * fr_matrix[8] + 3 * fr_matrix[9] - 2 * fr_matrix[10] - fr_matrix[11] - 3 * fr_matrix[12] + 3 * fr_matrix[13] - 2 * fr_matrix[14] - fr_matrix[15],
				4 * fr_matrix[0] - 4 * fr_matrix[1] + 2 * fr_matrix[2] + 2 * fr_matrix[3] - 4 * fr_matrix[4] + 4 * fr_matrix[5] - 2 * fr_matrix[6] - 2 * fr_matrix[7] + 2 * fr_matrix[8] - 2 * fr_matrix[9] + fr_matrix[10] + fr_matrix[11] + 2 * fr_matrix[12] - 2 * fr_matrix[13] + fr_matrix[14] + fr_matrix[15];

			double w_inv_i = 1.;
			for (int i = 0; i < 4; ++i) {
				double w_inv_j = 1.;
				for (int j = 0; j < 4; ++j) {
					surface.a_mat(i, j) *= w_inv_i * w_inv_j;
					w_inv_j *= width_inv;
				}
				w_inv_i *= width_inv;
			}
		}
	};


	double get_width(int level) const {
		return master_box_width_ / (1 << level); 
	}

	// returning false when container overflow
	// Will make sure that sample_ is a full quad tree.
	// When encountering existing box, will use addition.
	// Just addition: no box create box, have box: addition.
	// sampler can return nan when it is sure ther

	// You can imageine that , there are full boxes with zero delta surface applied to the layer.
	// the make sample action is simply trying to make correction to some zero vertices. by specifiying their absolute value.
	// 压 0 的时候， 把 boxes 声明大一些.  
	// Please make sure: 1, there is no surface >= level. 2, surfaces < level are freezed.


	struct ConvexSet2d {
		int a;
	};

	// 采样行为抽象成：
	// 对于整个空间， 有确定的采样函数， 这个函数具有以下形式:
	// f(x,y) = f_s0(x,y)* ((x,y) \in s0)
	//			f_s1(x,y)* ((x,y) \in s1)
	//			...
	//			f_sn(x,y)* ((x,y) \in sn)
	//			+ f_default(x,y)* ((x,y) !\in s0, s1,...sn)	  
	// where s0, s1, ... sn are convex sets.
	// 
	// 对于需要进行采样的格点set Q， 具有以下形式:
	// q(g) = all corners of boxes touching polygon g, 
	// Q = q(G).
	// G 称之为采样区:
	// 1,G is monotonically shrinking as level increases.
	// 2,G \in (\union s0, s1,...sn)  
	// 
	// s_domain_i = si \intersect G. 要求 s_domain_i convex.
	// 
	// 
	// 
	// We have algorithm:

	// for s in S:
	// 	 s_domain = s \intersect G.
	// 	 
	//	   for pt in q(s_domain):
	//       if pt in s:
	// 	       if sample(pt) not exist or virtual:
	//		     sample(pt) = f_s(pt)
	//		   else:
	// 	         sample(pt) += f_s(pt)
	//       else:
	//         if sample(pt) not exist:
	//           sample(pt) = f_default(x,y)
	//			 set sample(pt) virtual
	//		   else: 
	//           // do nothing.
	//
	// for points in q(s_domain), most of them are inside current s, those not inside s might be:
	// 1, sampled by other s; 2, never sampled by s and shoud be given f_default.

	// Since "pt in s" can be replaced by simply cutting  q(s_domain), the process is quicker.
	


	bool SampleRegion(const ConvexSet2d& s, const ConvexSet2d& s_domain, const std::function<double(double, double, double*, double*, double*)>& sampler) {
		// 1, All boxes whose father touches polygon.

		// 2, All 
		return true;
	}
	// Pre-requisite: 
	// 1, sample_surface below level are determined & populated. 
	bool AppendSamplesToLevel(const YMonotoneGridSet& points, int level, const std::function<double(double, double, double*, double*, double*)>&  sampler) {

		// When building boxes. We will need to follow delta expression. So we need background value.
		// Also, to sim the bounadry samples. make sure the verge boxes are not created from sampler but back ground.

		CHECK_GE(level, 0);
		CHECK_LT(level, sample_surfaces_.depth());
		double width = get_width(level);

		const std::vector<std::pair<int, int>> corner_dx_dy{ {0, 0, }, { 0,1 }, { 1,0 }, { 1,1 } };

		int min_yid = std::max(points.y_min_max.first, 0);
		int max_yid = std::min(points.y_min_max.second, 1 << level);

		for (int v_yid = min_yid; v_yid <= max_yid; ++v_yid) {
			int relative_yid = v_yid - points.y_min_max.first;
			CHECK_LT(relative_yid, points.x_min_max_list.size());
			const std::pair<int, int>& x_range = points.x_min_max_list[relative_yid];
			int min_xid = std::max(x_range.first, 0);
			int max_xid = std::min(x_range.second, 1 << level);

			for (int v_xid = min_xid; v_xid <= max_xid; ++v_xid) {
				// Make sample.
				double vx = v_xid * width;
				double vy = v_yid * width;
				
				// Get the absolute vertex sample.
				Vertex vertex;
				vertex.f = sampler(vx, vy, &vertex.df_dx, &vertex.df_dy, &vertex.d2f_dx_dy);
				DCHECK(!vertex.IsNaN());

				// Substract bgd. 
				Vertex bgd = GetMergedBackground(vx, vy, level - 1); 
				DCHECK(!bgd.IsNaN()); 
				vertex -= bgd;
				
				// Update/Add the sample to 4 boxes around.
				for (const std::pair<int, int>& dx_dy : corner_dx_dy) {
					int b_xid = v_xid - dx_dy.first;
					int b_yid = v_yid - dx_dy.second;

					if (!sample_surfaces_.IsInside(level, b_xid, b_yid)) {
						continue;
					}

					SampleSurface* sample_surface = sample_surfaces_.GetMutableElement(level, b_xid, b_yid);
					if (sample_surface != nullptr) {
						sample_surface->sample.Corner(dx_dy.first, dx_dy.second) += vertex;
					} else {
						SampleSurface new_sample_surface;
						new_sample_surface.sample.Corner(dx_dy.first, dx_dy.second) = vertex;
						if (!sample_surfaces_.AddElement(level, b_xid, b_yid, std::move(new_sample_surface))) {
							return false;
						}
					}
				}
			}
		}

		return true;
	}

	// Pre-requisite: 
	// You have finished AppendSamplesToLevel to current level
	// Samples below current level are populated.
	// Populate sample_surface.surface at this level.
	void PopulateSamplesAtLevel(int level) {
		CHECK_GE(level, 0);
		CHECK_LT(level, sample_surfaces_.depth());
		
		double width = get_width(level);
		const std::vector<std::pair<int, int>> corner_dx_dy{ {0, 0, }, { 0,1 }, { 1,0 }, { 1,1 } };
		
		const auto& begin_iter = sample_surfaces_.begin_of_level(level);
		const auto& end_iter = sample_surfaces_.end_of_level(level);
		for (auto iter = begin_iter; iter != end_iter; ++iter) {
			int unused, b_xid, b_yid;
			std::tie(unused, b_xid, b_yid) = iter.GetIndices();
			
			// Get background's vertices. 
			Sample bgd;
			for (const auto& dx_dy : corner_dx_dy) {
				bgd.Corner(dx_dy.first, dx_dy.second) =
					GetMergedBackground((b_xid + dx_dy.first) * width, (b_yid + dx_dy.second) * width, level - 1);
			}
			DCHECK(!bgd.HasNaN());

			iter->Populate(b_xid * width, b_yid * width, width, bgd); 
		}
	}
	

	// return false when container overflow, 
	// in such case, the entire action is abandoned and no change is made.
	// Upon success the function will make IsPopulated() false.
	bool Add(const QuadDividedCubicSurface& other_surface) {
		// Just add the deltas. 

		auto add_function = [](SampleSurface* self, const SampleSurface& other) {
			self->sample += other.sample;
		};

		bool result = sample_surfaces_.Merge(other_surface.sample_surfaces_, add_function);
		if (result) {
			// Successful Add will nullify the populated surfaces
			populated_ = false;
		}
		return result;
	}

	// Populate samples.
	void Populate() {
		if (populated_) {
			return;
		}
		for (int level = 0; level < sample_surfaces_.depth(); ++level) {
			PopulateSamplesAtLevel(level);
		}
		populated_ = true;
	}

	// Scale the surface.
	// The function will make IsPopulated() false.
	void Scale(double gain) {
		for (SampleSurface& sample_surface: sample_surfaces_) {
			sample_surface.sample *= gain; 
		}

		populated_ = false;
	}

	// Pre-requisite:
	// IsPopulated() == true.
	// The function won't change IsPopulated(), compared to Scale().
	void ScalePopulated(double gain) {
		CHECK(IsPopulated());
		for (SampleSurface& sample_surface : sample_surfaces_) {
			sample_surface.sample *= gain;
			sample_surface.surface.a_mat *= gain;
		}
	}
	

	// Pre-requisite: IsMerged()==true.
	// Call function to all vertex.
	// And populate

	// Saperate use of sample and populated result.
	// For each real sample, we need to know: 
	// The abs value modified <- abs value <- bgd surface.
	// The bgd value modified -> to get new delta.

	// For un-merged surface.
	// 1, level from 0 to depth -1, only update real samples's value to be modified abs value.
	// 	   during this step we can always get the correct bgd before call as there is no population.
	// 2, Now need to tranform the real samples to delta expression.
	//     level from 0 to depth -1, assume level before are populated using modified delta sample.
	//	get modified bgd, so you can get delta of the current level real sample. Update them to vertex and populate.
	// 3, You are all set.

	// The problem is, how severe is the effect of GetBackground, who uses many surfaces to get one vertex bgd.
	// Can we always get surfaces merged, for addition, add only samples, and re-populate?
	// Can we freeze before step 1, and only make sure fake samples are still fake? (show nan in delta).
	// Can we just maintain 2 surfaces to accelerate the whoel process?


	// Pre-requisite:
	// IsPopulated() == true.
	// The function won't change IsPopulated(), it will still be true.
	// The function allow user to change the content of eacch sample vertex 
	// by applying a modifier function.
	void ModifySample(const std::function<void(double /*x*/, double /*y*/, double* /*f*/, double* /*df_dx*/, double* /*df_dy*/, double* /*d2f_dx_dy*/)>& sample_modifier) {
		
		CHECK(IsPopulated());
		// "Real" samples will be updated to the after-function-call value.
		// After this step, each real sample vertex stores the absolute value & derivative info.
		for (auto iter = sample_surfaces_.begin(); iter != sample_surfaces_.end(); ++iter) {
			int level, b_xid, b_yid;
			std::tie(level, b_xid, b_yid) = iter.GetIndices();
			double width = get_width(level);

			for (const auto& dx_dy : corner_dx_dy) {
				Vertex& v = iter->sample.Corner(dx_dy.first, dx_dy.second);

				// Each not-NaN vertex will be modified.
				// Those NaN vertices means: follow background.
				if (v.IsNaN()) {
					continue;
				}

				double vx = (b_xid + dx_dy.first) * width;
				double vy = (b_yid + dx_dy.second) * width;
				 
				// Update v to its absolute value.
				v += GetMergedBackground(vx, vy, level - 1);
				
				// Update v to to be after function call.
				sample_modifier(vx, vy, &v.f, &v.df_dx, &v.df_dy, &v.d2f_dx_dy);
			}
		}


		// From 0 to depth-1, update each real sample vertex to delta value, and populate.
		for (int level = 0; level < sample_surfaces_.depth(); ++level) {
			// for those < level, surfaces are populated and samples are stored as delta value.
			double width = get_width(level);

			const auto& begin_iter = sample_surfaces_.begin_of_level(level);
			const auto& end_iter = sample_surfaces_.end_of_level(level);
			for (auto iter = begin_iter; iter != end_iter; ++iter) {
				int unused, b_xid, b_yid;
				std::tie(unused, b_xid, b_yid) = iter.GetIndices();

				// Know the background.
				Sample bgd;
				for (const auto& dx_dy : corner_dx_dy) {
					bgd.Corner(dx_dy.first, dx_dy.second) = 
						GetMergedBackground((b_xid + dx_dy.first) * width, (b_yid + dx_dy.second) * width, level - 1);
				}

				// For real samples, update their absolute value to delta value.
				for (const auto& dx_dy : corner_dx_dy) {
					Vertex& v = iter->sample.Corner(dx_dy.first, dx_dy.second);
					if (!v.IsNaN()) {
						v -= bgd.Corner(dx_dy.first, dx_dy.second);
					}
				}

				// With bgd and delta value, populate.
				iter->Populate(b_xid * width, b_yid * width, width, bgd);
			}
		}
	}



	// That the <=  max_freezed_level surfaces are all freezed version.
	Vertex GetMergedBackground(double x, double y, int max_freezed_level) {
		Vertex bgd = Vertex::Zero();

		if (max_freezed_level < 0) {
			return bgd;
		}

		const SampleSurface* sample_surface = sample_surfaces_.GetElementByPosition(x, y, master_box_width_, max_freezed_level);
		if (sample_surface == nullptr) {
			return bgd;
		}

		double unused;
		bgd.f = (sample_surface->surface).GetValueAndDerivative(x, y, &bgd.df_dx, &bgd.df_dy, &unused, &bgd.d2f_dx_dy, &unused);
		return bgd;
	}




	double master_box_width_ = 1.0;

	// 滿四叉樹.
	QuadTree<SampleSurface> sample_surfaces_;   
	bool populated_ = false;
	std::vector<const SampleSurface*> sample_surface_cache_;
};
