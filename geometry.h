#pragma once

#include <optional>

#include "check.h"



namespace geom {
	template<typename T>
	struct Vec2 {
	public:
		constexpr Vec2(T x, T y) :x(x), y(y) {}

		inline void Rotate90() {
T old_x = x;
x = -y;
y = x;
		}

		inline void Rotate180() {
			x = -x;
			y = -y;
		}

		inline void Rotate270() {
			T old_x = x;
			x = y;
			y = -x;
		}

		inline T Dot(const Vec2& other) const {
			return x * other.x + y * other.y;
		}

		inline T Cross(const Vec2& other) const {
			return  x * other.y - y * other.x;
		}

		inline Vec2 operator+(const Vec2& other) const {
			return Vec2{ x + other.x, y + other.y };
		}

		inline Vec2 operator-(const Vec2& other) const {
			return Vec2{ x - other.x, y - other.y };
		}

		

		inline Vec2& operator+=(const Vec2& other) {
			x += other.x;
			y += other.y;
			return *this;
		}

		inline Vec2& operator-=(const Vec2& other) {
			x -= other.x;
			y -= other.y;
			return *this;
		}

		T x = 0.;
		T y = 0.;
	};

	template<typename T, typename R>
	inline Vec2<T> operator*(R scalar, const Vec2<T>& vec) {
		return Vec2{ vec.x * scalar, vec.y * scalar };
	}

	template<typename T, typename R>
	inline Vec2<T> operator*(const Vec2<T>& vec, R scalar) {
		return Vec2{ vec.x * scalar, vec.y * scalar };
	}

	template <typename T>
	class HalfPlane {
	public:
		/// <summary>
		/// 
		/// </summary>
		/// <param name="start_point"></param>
		/// <param name="end_point"></param>
		/// <param name="closed"></param>
		/// <param name="on_the_left"></param>
		explicit HalfPlane(const Vec2<T>& start_point, const Vec2<T>& end_point, bool closed = true, bool on_the_left = true) :
			start(start_point),
			start_to_end(end_point - start),
			closed(closed),
			on_the_left(on_the_left) {
			DCHECK(start_to_end.x != 0 || start_to_end.y != 0);
		}

		bool IsInside(const Vec2<T>& point) const {
			T offset = start_to_end.Cross(point - start);
			if (closed) {
				return on_the_left ? offset >= 0 : offset <= 0;
			}
			else {
				return on_the_left ? offset > 0 : offset < 0;
			}
		}

		// Will neglect closed field.
		// epsilon > 0: critical inclusive.
		// epsilon <= 0: critical exclusive.
		bool IsInside(const Vec2<T>& point, T epsilon) const {
			T offset = start_to_end.Cross(point - start);
			return on_the_left ? offset > -epsilon : offset < epsilon;
		}

		// Always make sure:
		// CreateComplimentary().IsInside(p) == !IsInside(p).
		HalfPlane CreateComplimentary() const {
			return HalfPlane(!closed, !on_the_left, start, start_to_end);
		}

		// Return nullopt when parallel.
		std::optional<Vec2<T>> GetIntersect(const HalfPlane<T>& other) const {
			double delta = start_to_end.Cross(other.start_to_end);
			if (delta == 0) {
				return std::nullopt;
			}
			Vec2<T> s0_to_s1 = other.start - start;
			Vec2<T> result = start + (s0_to_s1.Cross(other.start_to_end) / delta) * start_to_end;
			return result;
		}


		bool IsParallel(const HalfPlane<T>& other) const {
			return start_to_end.Cross(other.start_to_end) == 0 || other.start_to_end.Cross(start_to_end) == 0;
		}

		// epsilon > 0: critical inclusive.
		// epsilon <= 0: critical exclusive.
		bool IsLeftBoundOfIntersectedFanRegion(const HalfPlane<T>& other, T epsilon) const {
			double delta = start_to_end.Cross(other.start_to_end);

			if (on_the_left == other.on_the_left) {
				return delta > -epsilon;
			}
			else {
				return delta < epsilon;
			}
		}

		// epsilon > 0: critical inclusive.
		// epsilon <= 0: critical exclusive.
		bool IsRightBoundOfIntersectedFanRegion(const HalfPlane<T>& other, T epsilon) const {
			return other.IsLeftBoundOfIntersectedFanRegion(*this, epsilon);
		}


		bool closed = true;
		bool on_the_left = true;
		Vec2<T> start;
		Vec2<T> start_to_end;

	private:
		constexpr HalfPlane(bool closed,
			bool on_the_left,
			Vec2<T> start,
			Vec2<T> start_to_end) : closed(closed), on_the_left(on_the_left), start(start), start_to_end(start_to_end) {}
	};
 
	class ConvexPolygonalRegionBase {
	public:
		static constexpr double kEpsilon = 1e-9;
		using Vec2d = Vec2<double>;
		// id iterates from edges with higher upper end point
		// to lower ones.
		// SHOULD avoid degenerated case. Such that consecutive
		// edges always have intersect. 
		// (No parallel or overlaping)
		virtual const HalfPlane<double>& Left(int id) const = 0;
		virtual const HalfPlane<double>& Right(int id) const = 0;

		// Must greater equal to 0.
		virtual int NumberOfLeftEdges() const = 0;

		// Must greater equal to 0.
		virtual int NumberOfRightEdges() const = 0;

		// nullopt indicates edge extends to infinity.
		std::optional<Vec2d> LowerEndPointOfLeftEdge(int id) const {
			const int& left_num = NumberOfLeftEdges();
			const int& right_num = NumberOfRightEdges();
			DCHECK_LT(id, left_num);
			DCHECK_GE(id, 0);

			const HalfPlane<double>& e = Left(id);
			if (id == left_num - 1) {
				if (right_num <= 0) {
					return std::nullopt;
				}
				else {
					const HalfPlane<double>& right_end = Right(right_num - 1);
					if (e.IsLeftBoundOfIntersectedFanRegion(right_end, -kEpsilon)) {
						return  right_end.GetIntersect(e);
					}
					else {
						return std::nullopt;
					}
				}
			} else {
				return Left(id+1).GetIntersect(e); 
			}
		}

		// nullopt indicates edge extends to infinity.
		std::optional<Vec2d> UpperEndPointOfLeftEdge0() const {
			const int& left_num = NumberOfLeftEdges();
			const int& right_num = NumberOfRightEdges(); 
			DCHECK_GT(left_num, 0);

			const HalfPlane<double>& e = Left(0); 
			if (right_num <= 0) {
				return std::nullopt;
			}
			else {
				const HalfPlane<double>& right_begin = Right(0);
				if (e.IsRightBoundOfIntersectedFanRegion(right_begin, -kEpsilon)) {
					return right_begin.GetIntersect(e);
				}
				else {
					return std::nullopt;
				}
			} 
		}

		// nullopt indicates edge extends to infinity.
		std::optional<Vec2d> LowerEndPointOfRightEdge(int id) const {
			const int& left_num = NumberOfLeftEdges();
			const int& right_num = NumberOfRightEdges();
			DCHECK_LT(id, right_num);
			DCHECK_GE(id, 0);

			const HalfPlane<double>& e = Right(id);
			if (id == right_num - 1) {
				if (left_num <= 0) {
					return std::nullopt;
				}
				else {
					const HalfPlane<double>& left_end = Left(left_num - 1);
					if (e.IsRightBoundOfIntersectedFanRegion(left_end, -kEpsilon)) {
						return left_end.GetIntersect(e);
					}
					else {
						return std::nullopt;
					}
				}
			}
			else {
				return Right(id + 1).GetIntersect(e);
			}
		}
		
		// nullopt indicates edge extends to infinity.
		std::optional<Vec2d> UpperEndPointOfRightEdge0() const {
			const int& left_num = NumberOfLeftEdges();
			const int& right_num = NumberOfRightEdges();
			DCHECK_GT(right_num, 0);

			const HalfPlane<double>& e = Right(0);
			if (left_num <= 0) {
				return std::nullopt;
			}
			else {
				const HalfPlane<double>& left_begin = Left(0);
				if (e.IsLeftBoundOfIntersectedFanRegion(left_begin, -kEpsilon)) {
					return left_begin.GetIntersect(e);
				}
				else {
					return std::nullopt;
				}
			}
		}

		// Return two vectors of index.
		// Each specifying the remaining edge ids of Left or Right.
		void RemoveEdgesOutside(const ConvexPolygonalRegionBase* other,
			std::vector<int>* new_left,
			std::vector<int>* new_right) const {
			DCHECK(new_left != nullptr);
			DCHECK(new_right != nullptr);
			DCHECK(other != nullptr);

			const int& left_num = NumberOfLeftEdges();
			const int& right_num = NumberOfRightEdges();
			DCHECK_GE(left_num, 0);
			DCHECK_GE(right_num, 0);
			// It is possible there is no left or right

			new_left->clear();
			new_right->clear();
			new_left->reserve(left_num);
			new_right->reserve(right_num);

			const int& other_left_num = other->NumberOfLeftEdges();
			const int& other_right_num = other->NumberOfRightEdges();

			// The iterator for sweeping line algorithms.
			struct EdgeGroup {
				bool is_left = false;
				// -1 indicates that this is an empty group.
				int id = -1;
				int num_edges = 0;
				std::optional<Vec2d> p, n;
				const ConvexPolygonalRegionBase* region = nullptr;

				// !empty().
				// Critical inclusive action.
				inline bool IsUpperEndInside(const EdgeGroup& other) const {
					DCHECK(!empty());
					if (other.empty()) {
						return true;
					}
					const HalfPlane<double>& e = edge();
					if (p) {
						// Inclusive.
						return other.edge().IsInside(*p, kEpsilon);
					}
					if (is_left) {
						// Inclusive.
						return e.IsLeftBoundOfIntersectedFanRegion(other.edge(), kEpsilon);
					}
					else {
						// Inclusive.
						return e.IsRightBoundOfIntersectedFanRegion(other.edge(), kEpsilon);
					}
				}

				// !empty()
				// Critical inclusive action.
				inline bool IsUpperEndOutside(const EdgeGroup& other) const {
					DCHECK(!empty());
					if (other.empty()) {
						return false;
					}
					const HalfPlane<double>& e = edge();
					if (p) {
						// Inclusively outside.
						return !other.edge().IsInside(*p, -kEpsilon);
					}
					if (is_left) {
						// Inclusive.
						return e.IsRightBoundOfIntersectedFanRegion(other.edge(), kEpsilon);
					}
					else {
						// Inclusive.
						return e.IsLeftBoundOfIntersectedFanRegion(other.edge(), kEpsilon);
					}
				}

				// !empty().
				const HalfPlane<double>& edge() const {
					DCHECK(!empty());
					DCHECK(region != nullptr);

					return is_left ? region->Left(id) : region->Right(id);
				}

				void Proceed() {
					if (id < 0) {
						return;
					}
					else if (id == num_edges - 1) {
						// no more next edge.
						id = -1;
					}
					else {
						id += 1;
						std::swap(p, n);
						DCHECK(region != nullptr);
						n = is_left ? region->LowerEndPointOfLeftEdge(id) : region->LowerEndPointOfRightEdge(id);
					}
				}

				inline bool empty() const {
					return id < 0;
				}

				// !empty().
				// Critical inclusive action.
				inline bool HasIntersectWith(const EdgeGroup& other) const {
					DCHECK(!empty());
					if (other.empty() || empty()) {
						return false;
					}

					std::optional<Vec2d> intersect = other.edge().GetIntersect(edge());
					if (!intersect) {
						return false;
					}
					// intersect should be inside p, n;
					if (p && intersect->y > p->y + kEpsilon) {
						return false;
					}
					if (n && intersect->y < n->y - kEpsilon) {
						return false;
					}
					// intersect should be inside other's p, n;
					if (other.p && intersect->y > other.p->y + kEpsilon) {
						return false;
					}
					if (other.n && intersect->y < other.n->y - kEpsilon) {
						return false;
					}
					return true;
				}

				// !empty() & !other.empty().
				// Critical exclusive action.
				bool IsLowerEndPointHigherThan(const EdgeGroup& other) const {
					DCHECK(!empty());
					DCHECK(!other.empty());

					// For horizontal edge. 
					// Deem that it is slightly tilted, so it reaches y = -inf.
					if (!n) {
						// me extends to inf.
						return false;
					}
					if (!other.n) {
						// other extends to inf.
						return true;
					}
					return n->y > other.n->y;
				}
			};

			// left, right, other_left, other_right;
			EdgeGroup group[4];

			// Brother is the other group of edges in the same polyhedral region.
			// same_dir and diff_dir are the groups of edges in the other region.
			// gid of me = {0, 1, 2, 3}
			// gid of brother = {1, 0, 3, 2}
			const int gid_of_same_dir[] = { 2,3,0,1 };
			const int gid_of_diff_dir[] = { 3,2,1,0 };


			if (left_num > 0) {
				EdgeGroup& g = group[0];
				g.is_left = true;
				g.id = 0;
				g.num_edges = left_num;
				g.p = UpperEndPointOfLeftEdge0();
				g.n = LowerEndPointOfLeftEdge(0);
				g.region = this;
			}// otherwise empty group.
			if (right_num > 0) {
				EdgeGroup& g = group[1];
				g.is_left = false;
				g.id = 0;
				g.num_edges = right_num;
				g.p = UpperEndPointOfRightEdge0();
				g.n = LowerEndPointOfRightEdge(0);
				g.region = this;
			}// otherwise empty group.
			if (other_left_num > 0) {
				EdgeGroup& g = group[2];
				g.is_left = true;
				g.id = 0;
				g.num_edges = other_left_num;
				g.p = other->UpperEndPointOfLeftEdge0();
				g.n = other->LowerEndPointOfLeftEdge(0);
				g.region = other;
			}// otherwise empty group.
			if (other_right_num > 0) {
				EdgeGroup& g = group[3];
				g.is_left = false;
				g.id = 0;
				g.num_edges = other_right_num;
				g.p = other->UpperEndPointOfRightEdge0();
				g.n = other->LowerEndPointOfRightEdge(0);
				g.region = other;
			}// otherwise empty group.

			// left, right.
			int last_added_edge[2] = { -1,-1 };
			auto push_result = [&group, &last_added_edge, &new_left, &new_right](int gid) {
				if (gid < 0 || gid >1) {
					// Only accept left, right.
					return;
				}
				if (group[gid].empty()) {
					// Group empty.
					return;
				}
				if (last_added_edge[gid] >= group[gid].id) {
					// Should have been already handled.
					return;
				}

				last_added_edge[gid] = group[gid].id;

				if (gid == 0) {
					new_left->push_back(group[gid].id);
				}
				else if (gid == 1) {
					new_right->push_back(group[gid].id);
				}
			};

			int max_iteration_protector = left_num + right_num + other_left_num + other_right_num + 5;
			for (int i = 0; i < max_iteration_protector; ++i) {
				// Find the group with highest lower end point.
				int cur_gid = -1;
				for (int gid = 0; gid < 4; ++gid) {
					if (group[gid].empty()) {
						continue;
					}
					if (cur_gid < 0) {
						cur_gid = gid;
					}
					else if (group[gid].IsLowerEndPointHigherThan(group[cur_gid])) {
						cur_gid = gid;
					}
				}

				if (cur_gid < 0) {
					// all empty.
					break;
				}
				DCHECK_GE(cur_gid, 0);
				DCHECK_LT(cur_gid, 4);
				// suppose 0

				const EdgeGroup& cur = group[cur_gid];
				const int same_dir_gid = gid_of_same_dir[cur_gid];
				const int diff_dir_gid = gid_of_diff_dir[cur_gid];

				const EdgeGroup& same_dir = group[same_dir_gid];
				const bool upper_end_inside_same_dir = cur.IsUpperEndInside(same_dir);
				const bool upper_end_outside_same_dir = cur.IsUpperEndOutside(same_dir);
				const bool intersect_with_same_dir = cur.HasIntersectWith(same_dir);

				const EdgeGroup& diff_dir = group[diff_dir_gid];
				const bool upper_end_inside_diff_dir = cur.IsUpperEndInside(diff_dir);
				const bool upper_end_outside_diff_dir = cur.IsUpperEndOutside(diff_dir);
				const bool intersect_with_diff_dir = cur.HasIntersectWith(diff_dir);

				if (upper_end_inside_same_dir && upper_end_inside_diff_dir) {
					push_result(cur_gid);
				}

				if (intersect_with_diff_dir) {
					if (upper_end_outside_diff_dir) {
						push_result(cur_gid);
						push_result(diff_dir_gid);
					}
				}

				if (intersect_with_same_dir) {
					if (upper_end_outside_same_dir) {
						push_result(cur_gid);
					}
					if (upper_end_inside_same_dir) {
						push_result(same_dir_gid);
					}
				}

				group[cur_gid].Proceed();
			}
		}	

		// TODO(huaiyuan): get intersection.
		
		bool IsInside(const Vec2d& point) const {
			int left_num = NumberOfLeftEdges(); 
			for (int i = 0; i < left_num; ++i) {
				if (!Left(i).IsInside(point)) {
					return false;
				}
			}
			int right_num = NumberOfRightEdges();
			for (int i = 0; i < right_num; ++i) {
				if (!Right(i).IsInside(point)) {
					return false;
				}
			}
			return true;
		}
	};

	using HalfPlaneD = HalfPlane<double>;
	class ConvexPolyonalRegionInVector : public ConvexPolygonalRegionBase {
	public:
		explicit ConvexPolyonalRegionInVector(std::vector<HalfPlaneD> left, std::vector<HalfPlaneD> right) :
			left_(std::move(left)),
			right_(std::move(right)) {}

		const HalfPlaneD& Left(int id) const override {
			DCHECK_LT(id, left_.size());
			DCHECK_GE(id, 0);
			return left_[id];
		}
		
		const HalfPlaneD& Right(int id) const override {
			DCHECK_LT(id, right_.size());
			DCHECK_GE(id, 0);
			return right_[id];
		}

		// TODO(huaiyuan): INT_MAX saturate cast.
		int NumberOfLeftEdges() const override {
			return static_cast<int>(left_.size());
		}

		// TODO(huaiyuan): INT_MAX saturate cast.
		int NumberOfRightEdges() const override {
			return static_cast<int>(right_.size());
		}


	private:
		std::vector<HalfPlaneD> left_;
		std::vector<HalfPlaneD> right_;
	};


	// 四分树 特制 凸包 求内部点方案。
	// 锁定栅格顶点， 确保 不同level 相同 位置结果。 30 min.
	// 缩减边的数量的时候， 祈祷不要所有边都在一个小的box 内部。（这个应该能证明， 最坏是 depth * num_elements）
	// 特制 求交 方案: 去除box 的边（防止新增约束乱序）， 对于不确定的约束尽量保留。1 hour + 30min UT
	// 两边重合怎么破：1， 判断重合. 2, 扔掉box的。special design +30min
	// 满四叉树 的性质， 利用这个凸包左右边的严格性， 我们可以更好的找出所有内点， + intersection error + sweeping line. + 1 hour.
	// 休息： 一小时。
	
	using HalfPlaneD = HalfPlane<double>;
} // namespace

