#include "quad_tree_test.h"

#include <stdlib.h>
#include <unordered_map>
#include <iostream>

namespace test {
	namespace test_quad_tree {
		// Random create node, delete node. CHECK existence. Function as a map.
		void TestAsAMap() {
			int depth = 7;
			int N = 1000000;
			QuadTree<double> tree(depth);

			struct hash_int_pair {
				size_t operator()(const std::pair<int, int>& p) const
				{
					size_t hash1 = std::hash<int>{}(p.first);
					size_t hash2 = std::hash<int>{}(p.second);
					return hash1 ^ hash2;
				}
			};
			std::vector<std::unordered_map<std::pair<int, int>, double, hash_int_pair>> elements_at_level(depth);

			srand(0);

			std::vector<int> power_of_two(depth + 1, 1);
			for (int i = 1; i < depth + 1; ++i) {
				power_of_two[i] = 2 * power_of_two[i - 1];
			}

			int log_point = N/10 ;
			for (int i = 0; i < N; ++i) {
				LOG_IF(ERROR, /*i>219900 && */ i% log_point == 0) << i << "/" << N;

				int level = rand() % depth;
				int xid = rand() % power_of_two[level];
				int yid = rand() % power_of_two[level];

				CHECK(tree.IsInside(level, xid, yid));

				auto key = std::make_pair(xid, yid);
				if (elements_at_level[level].count(key) > 0) {
					// CHECK existance and equality.
					const double* ele = tree.GetElement(level, xid, yid);
					CHECK(ele!=nullptr);
					CHECK_EQ(*ele, elements_at_level[level][key]);

					// Delete it or not delete it.
					if (rand() % 2 == 0) {
						elements_at_level[level].erase(key);
						tree.RemoveElement(level, xid, yid);
					}
				}
				else {
					const double* ele = tree.GetElement(level, xid, yid);
					CHECK(ele == nullptr);

					// Add it.
					double val = rand() % 1000;
					elements_at_level[level][key] = val;
					tree.AddElement(level, xid, yid, val);
				}
			}
		}

		// Generate elements from positions.
		void TestGeometricFunctionality() {
			constexpr int kDepth = 6;
			

			srand(0);
			QuadTree<std::tuple<int, int, int>> indices_tree(kDepth);

			std::vector<int> num_elements_at_level(kDepth, 0);

			// Post some random elements.
			for (int level = 0; level < kDepth; level++) {
				int num_boxes = std::pow(2, level);
				for (int xid = 0; xid < num_boxes; ++xid) {
					for (int yid = 0; yid < num_boxes; ++yid) {
						if (rand() % 2 == 0 && level != 3) {
							num_elements_at_level[level] += 1;
							indices_tree.AddElement(level, xid, yid, std::make_tuple(level, xid, yid));
						}
					}
				}
			}

			// Randomly delete 1/3 elements.
			for (int level = 0; level < kDepth; level++) {
				int num_boxes = std::pow(2, level);
				for (int xid = 0; xid < num_boxes; ++xid) {
					for (int yid = 0; yid < num_boxes; ++yid) {
						if (rand() % 3 == 0 && indices_tree.GetElement(level, xid, yid)!=nullptr) {
							num_elements_at_level[level] -= 1;
							indices_tree.RemoveElement(level, xid, yid);
						}
					}
				}
			}

			// CHECK iterator conectivity and correctness. 
			for (int level = 0; level < kDepth; level++) {
				CHECK_EQ(indices_tree.size(level), num_elements_at_level[level]);
				int num_elements_by_iterating = 0;
				for (auto iter = indices_tree.begin_of_level(level); iter != indices_tree.end_of_level(level); ++iter) {
					num_elements_by_iterating += 1;
				}
				CHECK_EQ(num_elements_by_iterating, num_elements_at_level[level]);
			}

			// Check 
			int N = 1000000;
			int div = 10000;
			int log_point = N / 10;
			
			for (int i =0; i <N ;++i) {
				LOG_IF(ERROR,  i % log_point == 0) << i << "/" << N ;

				double x = rand() % div * 1.0 / div;
				double y = rand() % div * 1.0 / div;

				QuadTree<std::tuple<int, int, int>>::ConstElementReferenceArray	result = indices_tree.GetElementsByPosition(x, y);

				for (int i = 0; i < result.num_elements; ++i) {
					const std::tuple<int, int, int>& ele = *result.elements[i];
					int level = std::get<0>(ele);
					int num_boxes = 1 << level;
					CHECK_EQ(static_cast<int>(x * num_boxes), std::get<1>(ele));
					CHECK_EQ(static_cast<int>(y * num_boxes), std::get<2>(ele));
				}

				auto leaf = indices_tree.GetElementByPosition(x, y);
				if (leaf != nullptr) {
					int level = std::get<0>(*leaf);
					int num_boxes = 1 <<level;
					CHECK_EQ(static_cast<int>(x * num_boxes), std::get<1>(*leaf));
					CHECK_EQ(static_cast<int>(y * num_boxes), std::get<2>(*leaf));
				}	
			}
		}

		// Random create double trees A,B,C,D ...
		// Merge by A + B + C + D...
		void TestMerge() {
			int depth = 7;
			int N = 5000;

			struct hash_int_pair {
				size_t operator()(const std::pair<int, int>& p) const
				{
					size_t hash1 = std::hash<int>{}(p.first);
					size_t hash2 = std::hash<int>{}(p.second);
					return hash1 ^ hash2;
				}
			};
			std::vector<std::unordered_map<std::pair<int, int>, double, hash_int_pair>> elements_at_level(depth);

			srand(0);


			QuadTree<double> tree1(depth);
			QuadTree<double> tree2(depth);

			std::vector<int> power_of_two(depth + 1, 1);
			for (int i = 1; i < depth + 1; ++i) {
				power_of_two[i] = 2 * power_of_two[i - 1];
			}

			for (int i = 0; i < N; ++i) {

				int level = rand() % depth;
				int xid = rand() % power_of_two[level];
				int yid = rand() % power_of_two[level];

				CHECK(tree1.IsInside(level, xid, yid));

				auto key = std::make_pair(xid, yid);

				if (tree1.GetElement(level, xid, yid) == nullptr) {
					double val = rand() % 1000;

					if (elements_at_level[level].count(key) < 1) {
						elements_at_level[level][key] = 0;
					}

					elements_at_level[level][key] += val;
					tree1.AddElement(level, xid, yid, val);
				}
			}

			for (int i = 0; i < N; ++i) {

				int level = rand() % depth;
				int xid = rand() % power_of_two[level];
				int yid = rand() % power_of_two[level];

				CHECK(tree1.IsInside(level, xid, yid));

				auto key = std::make_pair(xid, yid);

				if (tree2.GetElement(level, xid, yid) == nullptr) {
					double val = rand() % 1000;
					if (elements_at_level[level].count(key) < 1) {
						elements_at_level[level][key] = 0;
					}
					elements_at_level[level][key] += val;
					tree2.AddElement(level, xid, yid, val);
				}
			}

			tree1.Merge(tree2, [](double* self, const double& other) {*self += other; });
			for (int level = 0; level < elements_at_level.size();level++) {
				for (const auto& k_v : elements_at_level[level]) {
					CHECK(tree1.IsInside(level, k_v.first.first, k_v.first.second));
					const double* ele = tree1.GetElement(level,k_v.first.first, k_v.first.second);

					CHECK(ele != nullptr);
					CHECK_EQ(*ele, k_v.second);
				}
			}


		}

	}

	void TestQuadTree() {
		LOG(ERROR) << "==Testing Map Functionality==";
		test_quad_tree::TestAsAMap();
		LOG(ERROR) << "==Testing Complete==";

		LOG(ERROR) << "==Testing Geometric Functionality==";
		test_quad_tree::TestGeometricFunctionality();
		LOG(ERROR) << "==Testing Complete==";

		LOG(ERROR) << "==Testing Merge==";
		test_quad_tree::TestMerge();
		LOG(ERROR) << "==Testing Complete==";
	}

} // namespace test