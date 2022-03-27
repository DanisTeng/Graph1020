#pragma once

#include <algorithm>
#include <optional>
#include <vector>
#include <array>
#include <functional>

#include "geometry.h"
#include "check.h"


template <class T>
class QuadTree {
private:
	struct Node {
		int total_num_elements_in_tree = 0;
		int father = 0;
		int id_as_a_child = 0;
		int element_info_id = -1;
		int children[4] = { 0 };
	};

	struct ElementInfo {
		int level = 0;
		int xid = 0;
		int yid = 0;
		// Link elements in a bidirectional linked list.
		// Elements in smaller level will appear earlier.
		// This can facilitate by-level iterating.
		// kHead -> L01->L02..L0n_0->L11->L12->...Lkn_k -> kTail
		int prev = -1;
		int next = -1;
		int node_id = 0;
		T element;
	};
public:
	// Limited by the fact that xid, yid uses int.
	// Limited by the precision of double: 52 bit.
	static constexpr int kMaxDepth = 28;

	struct ConstElementReferenceArray {
		int num_elements = 0;
		std::array<const T*, kMaxDepth> elements;
	};

	struct ElementReferenceArray {
		int num_elements = 0;
		std::array<T*, kMaxDepth> elements;
	};

	struct ElementInfoIndexArray {
		int num_elements = 0;
		std::array<int, kMaxDepth> elements;
	};

	struct iterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using difference_type = std::ptrdiff_t;
		using value_type = T;
		using pointer = T*;
		using reference = T&;

		explicit iterator(std::vector<ElementInfo>* elements, int id = -1) : elements_(CHECK_NOTNULL(elements)), id_(id) {}

		reference operator*() const {
			CHECK(id_ >= 0 && id_ < elements_->size());
			return (*elements_)[id_].element;
		}
		pointer operator->() {
			CHECK(id_ >= 0 && id_ < elements_->size());
			return &(*elements_)[id_].element;
		}

		inline std::tuple<int, int, int> GetIndices() const {
			CHECK(id_ >= 0 && id_ < elements_->size());
			const ElementInfo& info = (*elements_)[id_];
			return std::make_tuple(info.level, info.xid, info.yid);
		}

		// Prefix increment
		iterator& operator++() {
			CHECK(id_ >= 0 && id_ < elements_->size());
			id_ = (*elements_)[id_].next; return *this;
		}

		// Postfix increment
		iterator operator++(int) { iterator tmp = *this; ++(*this); return tmp; }

		friend bool operator== (const iterator& a, const iterator& b) { return a.id_ == b.id_; }
		friend bool operator!= (const iterator& a, const iterator& b) { return a.id_ != b.id_; }

	private:
		// Not owned.
		std::vector<ElementInfo>* elements_ = nullptr;
		int id_ = -1;
		friend class QuadTree;
	};

	struct const_iterator {
	public:
		using iterator_category = std::forward_iterator_tag;
		using difference_type = std::ptrdiff_t;
		using value_type = T;
		using pointer = const T*;
		using reference = const T&;

		explicit const_iterator(const std::vector<ElementInfo>* elements, int id = -1) : elements_(CHECK_NOTNULL(elements)), id_(id) {}

		reference operator*() const {
			CHECK(id_ >= 0 && id_ < elements_->size());
			return (*elements_)[id_].element;
		}
		pointer operator->() const {
			CHECK(id_ >= 0 && id_ < elements_->size());
			return &(*elements_)[id_].element;
		}

		inline std::tuple<int, int, int> GetIndices() const {
			CHECK(id_ >= 0 && id_ < elements_->size());
			const ElementInfo& info = (*elements_)[id_];
			return std::make_tuple(info.level, info.xid, info.yid);
		}

		// Prefix increment
		const_iterator& operator++() {
			CHECK(id_ >= 0 && id_ < elements_->size());
			id_ = (*elements_)[id_].next; return *this;
		}

		// Postfix increment
		const_iterator operator++(int) { iterator tmp = *this; ++(*this); return tmp; }

		friend bool operator== (const const_iterator& a, const const_iterator& b) { return a.id_ == b.id_; }
		friend bool operator!= (const const_iterator& a, const const_iterator& b) { return a.id_ != b.id_; }

	private:
		// Not owned.
		const std::vector<ElementInfo>* elements_ = nullptr;
		int id_ = -1;
		friend class QuadTree;
	};

	// A box is always identified using 3 integers:
	// level, xid, yid.
	// The definition of them are as below:
	// (xid, yid):
	//        ===level 0===         ===level 1===
	//      _________________     _________________
	// 	   |                 |   |        |        |
	// 	   |                 |   | (0,1)  | (1,1)  |
	//     |                 |   |        |        |
	// 	   |     (0, 0)      |   |-----------------|
	//     |                 |   |        |        |
	//     |                 |   | (0,0)  | (0,1)  |
	// 	   |_________________|   |________|________|
	//
	// QuadTree is a container of user specified elements
	// with each one associates with a box.
	explicit QuadTree(int depth) :
		// Limited by the fact that "node index uses int".
		max_num_elements_(INT_MAX / depth - 2),
		depth_(depth) {

		CHECK_GT(depth_, 0);
		CHECK_LE(depth_, kMaxDepth);

		max_id_upper_bound_inv_ = 1 << (depth_ - 1);
		max_id_upper_bound_inv_ = 1.0 / max_id_upper_bound_inv_;
		CHECK_GT(max_id_upper_bound_inv_, 0.0);


		num_elements_at_level_.resize(depth_, 0);
		nodes_.emplace_back(); // The "nullptr"
		element_infos_.emplace_back(); // Head
		element_infos_.emplace_back(); // Tail
		element_infos_[kHead].next = kTail;
		element_infos_[kTail].prev = kHead;

		begins_at_level_.resize(depth_, -1);
		prev_ends_at_level_.resize(depth_, -1);
	}


	// The xid & yid upper bound equals 2^level.
	// The function returns false when xid or yid is out of that bound or 
	// given level is out greater than depth()-1.
	inline bool IsInside(int level, int xid, int yid) const {
		if (level < 0 || level >= depth_) {
			return false;
		}
		int id_upper_bound = (1 << level);

		return xid >= 0 && xid < id_upper_bound&& yid >= 0 && yid < id_upper_bound;
	}


	inline static bool IsPositionInside(double x, double y, double root_box_width) {
		return  (x >= 0 && x <= root_box_width && y >= 0 && y <= root_box_width);
	}


	geom::AABox2d BoxAt(int level, int xid, int yid, double unit_box_width) const {

		const int gain = 1 << (depth_ - 1 - level);

		return geom::AABox2d{ (xid * gain) * unit_box_width,
							((xid + 1) * gain) * unit_box_width ,
							(yid * gain) * unit_box_width ,
							((yid + 1) * gain) * unit_box_width };
	}



	double RootBoxWidthToUnitBoxWidth(double root_box_width) const {
		return root_box_width / (1 << (depth_ - 1));
	}

	double UnitBoxWidthToRootBoxWidth(double unit_box_width) const {
		return unit_box_width  * (1 << (depth_ - 1));
	}


	// Pre-condition: 
	// IsInside(level, xid, yid) &&
	// GetElement(level, xid, yid) == nullptr.
	// Add element to the given box.
	// Please also check: 
	// size()+1 <= max_num_elements() 
	// so that it won't return false.
	// O(depth());
	// true -> new element added.
	// false -> container overflow.
	bool AddElement(int level, int xid, int yid, T element) {
		CHECK(IsInside(level, xid, yid));

		if (num_elements_ + 1 > max_num_elements_) {
			return false;
		}

		int* pnode = &root_;
		int node = 0;

		int father = 0;
		int id_as_a_child = 0;

		int relative_xid = xid;
		int relative_yid = yid;

		for (int l = 0; l <= level; ++l) {
			// Update node.
			if (*pnode == 0) {
				// Create If pnode is empty.
				int new_node = static_cast<int>(nodes_.size());
				*pnode = new_node;
				// Emplace back later to avoid int* nullification.
				// pnode is nullified.
				nodes_.emplace_back();

				node = new_node;

				nodes_[node].father = father;
				nodes_[node].id_as_a_child = id_as_a_child;
			}
			else {
				// Navigate to pnode.
				node = *pnode;
			}
			// node != 0;
			// pnode is nullified.

			if (l == level) {
				CHECK(nodes_[node].element_info_id < 0);
				int new_element = static_cast<int>(element_infos_.size());

				int next_non_empty_level = level + 1;
				while (next_non_empty_level < depth_ && begins_at_level_[next_non_empty_level] < 0) {
					next_non_empty_level++;
				}
				int next_ele = next_non_empty_level >= depth_ ? kTail : begins_at_level_[next_non_empty_level];
				int prev_ele = element_infos_[next_ele].prev;
				CHECK(prev_ele != -1);
				CHECK(next_ele != -1);

				element_infos_[prev_ele].next = new_element;
				element_infos_[next_ele].prev = new_element;

				prev_ends_at_level_[level] = new_element;
				if (num_elements_at_level_[level] == 0) {
					begins_at_level_[level] = new_element;
				}
				num_elements_at_level_[level]++;
				num_elements_++;

				element_infos_.push_back(
					{
					level,
					xid,
					yid,
					prev_ele,
					next_ele,
					node,
					std::move(element)
					}
				);
				nodes_[node].element_info_id = new_element;
			}
			else {
				const int& mid = 1 << level - 1 - l;
				father = node;

				if (relative_xid >= mid) {
					relative_xid -= mid;
					if (relative_yid >= mid) {
						relative_yid -= mid;
						id_as_a_child = 3;
					}
					else {
						id_as_a_child = 2;
					}
				}
				else {
					if (relative_yid >= mid) {
						relative_yid -= mid;
						id_as_a_child = 1;
					}
					else {
						id_as_a_child = 0;
					}
				}
				pnode = &nodes_[node].children[id_as_a_child];
			}
		}

		while (node != 0) {
			nodes_[node].total_num_elements_in_tree += 1;
			node = nodes_[node].father;
		}

		return true;
	}

	// Pre-condition: IsInside(level, xid, yid).
	// Remove element if exist on given box.
	// Otherwise do nothing.
	// O(depth());
	// true -> existing element removed
	// false -> element not exist
	bool RemoveElement(int level, int xid, int yid) {
		CHECK(IsInside(level, xid, yid));
		int node = root_;
		bool removed = false;

		for (int l = 0; l <= level; ++l) {
			if (node == 0) {
				break;
			}
			else if (l == level) {
				if (nodes_[node].element_info_id >= 0) {
					int removed_element = nodes_[node].element_info_id;
					int next_ele = element_infos_[removed_element].next;
					int prev_ele = element_infos_[removed_element].prev;

					element_infos_[prev_ele].next = next_ele;
					element_infos_[next_ele].prev = prev_ele;

					CHECK_GT(num_elements_at_level_[level], 0);
					if (num_elements_at_level_[level] == 1) {
						begins_at_level_[level] = -1;
						prev_ends_at_level_[level] = -1;
					}
					else {
						if (removed_element == begins_at_level_[level]) {
							begins_at_level_[level] = next_ele;
						}
						else if (removed_element == prev_ends_at_level_[level]) {
							prev_ends_at_level_[level] = prev_ele;
						}
					}
					num_elements_at_level_[level]--;
					num_elements_--;

					// Erase object from the vector.
					// By swapping it with last element.
					int last_id = static_cast<int>(element_infos_.size() - 1);
					if (last_id != removed_element) {
						const ElementInfo& last = element_infos_.back();

						// All who points to last element shall be notified
						element_infos_[last.prev].next = removed_element;
						element_infos_[last.next].prev = removed_element;
						if (begins_at_level_[last.level] == last_id) {
							begins_at_level_[last.level] = removed_element;
						}
						if (prev_ends_at_level_[last.level] == last_id) {
							prev_ends_at_level_[last.level] = removed_element;
						}
						nodes_[last.node_id].element_info_id = removed_element;
						// that it has been moved to the position of removed_element

						// Personally I believe copy is not avoidable here.
						// The effective way of deleting a large element 
						// should be using external storage.
						// (Or simply, don't delete).
						element_infos_[removed_element] = std::move(element_infos_[last_id]);
					}

					element_infos_.pop_back();

					nodes_[node].element_info_id = -1;
					removed = true;
				}
				else {
					removed = false;
				}
			}
			else {
				const int& mid = 1 << level - 1 - l;

				int id_as_a_child = -1;
				if (xid >= mid) {
					xid -= mid;

					if (yid >= mid) {
						yid -= mid;
						id_as_a_child = 3;
					}
					else {
						id_as_a_child = 2;
					}
				}
				else {
					if (yid >= mid) {
						yid -= mid;
						id_as_a_child = 1;
					}
					else {
						id_as_a_child = 0;
					}
				}
				node = nodes_[node].children[id_as_a_child];
			}
		}

		if (removed) {
			int node_reverse = node;

			// Update num of elements in sub tree.
			while (node_reverse != 0) {
				nodes_[node_reverse].total_num_elements_in_tree -= 1;
				node_reverse = nodes_[node_reverse].father;
			}


			node_reverse = node;
			while (node_reverse != 0) {
				if (nodes_[node_reverse].total_num_elements_in_tree < 1) {
					// assert child less.
					node_reverse = DeleteChildLessNode(node_reverse);
				}
				else {
					break;
				}
			}

		}

		return removed;
	}

	// Pre-condition: IsInside(level, xid, yid).
	// Return the element's pointer if it exists on given box.
	// Otherwise nullptr.
	const T* GetElement(int level, int xid, int yid) const {
		int element_id = ElementInfoIdFromIndices(level, xid, yid);

		return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
	}

	// Pre-condition: IsInside(level, xid, yid).
	// Return the element's pointer if it exists on given box.
	// Otherwise nullptr.
	T* GetMutableElement(int level, int xid, int yid) {
		int element_id = ElementInfoIdFromIndices(level, xid, yid);
		return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
	}


	// TODO(huaiyuan): Current x,y based functions are not accurate.
	// The box of the result element [x0,x1) * [y0,y1) might not contain the given x,y.
	// Use Pos2Id for precise implementation: Grid class will tell you xid, yid.
	// Then: when enumerating xid, yid, you can use its bits to judge left or right. It looks Cooler stabler than double +-

	// TODO(huaiyuan): please use unit_width instead of root box width.

	// Return the element with smallest box that contains the given point.
	// Return nullptr if no box(level constrained by max_level) is containing it.
	// root_box_width: Will assume box(0,0,0) occupies the intersection of 
	// x \in (0, root_box_width) and y \in (0, root_box_width).
	const T* GetElementByPosition(double x, double y, double root_box_width = 1., int max_level = INT_MAX) const {
		int element_id = ElementInfoIdFromPosition(x, y, max_level, root_box_width);
		return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
	}

	// Return the element with smallest box that contains the given point.
	// Return nullptr if no box(level constrained by max_level) is containing it.
	// root_box_width: Will assume box(0,0,0) occupies the intersection of 
	// x \in (0, root_box_width) and y \in (0, root_box_width).
	T* GetMutableElementByPosition(double x, double y, double root_box_width = 1., int max_level = INT_MAX) {
		int element_id = ElementInfoIdFromPosition(x, y, max_level, root_box_width);
		return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
	}

	ConstElementReferenceArray GetElementsByPosition(double x, double y, double root_box_width = 1., int max_level = INT_MAX) const {
		ConstElementReferenceArray result;
		const ElementInfoIndexArray element_info_ids = ElementInfoIdsFromPosition(x, y, root_box_width, max_level);

		result.num_elements = element_info_ids.num_elements;
		for (int i = 0; i < element_info_ids.num_elements; ++i) {
			result.elements[i] = &element_infos_[element_info_ids.elements[i]].element;
		}
		return result;
	};

	ElementReferenceArray GetMutableElementsByPosition(double x, double y, double root_box_width = 1., int max_level = INT_MAX) {
		ElementReferenceArray result;
		const ElementInfoIndexArray element_info_ids = ElementInfoIdsFromPosition(x, y, root_box_width, max_level);

		result.num_elements = element_info_ids.num_elements;
		for (int i = 0; i < element_info_ids.num_elements; ++i) {
			result.elements[i] = &element_infos_[element_info_ids.elements[i]].element;
		}
		return result;
	};


	double depth() const {
		return depth_;
	}

	// Pre-condition:  0 <= level < depth().
	// Return the number of elements at certain level.
	size_t size(int level) const {
		CHECK(level >= 0 && level < depth_);
		return num_elements_at_level_[level];

	}
	size_t size() const {
		return num_elements_;
	}

	iterator begin() {
		return iterator{ &element_infos_, element_infos_[kHead].next };
	}

	const_iterator cbegin() const {
		return const_iterator{ &element_infos_, element_infos_[kHead].next };
	}

	iterator end() {
		return iterator{ &element_infos_, kTail };
	}

	const_iterator cend() const {
		return const_iterator{ &element_infos_, kTail };
	}

	iterator begin_of_level(int level) {
		CHECK(level >= 0 && level < depth_);
		if (begins_at_level_[level] < 0) {
			return iterator{ &element_infos_ };
		}
		else {
			return iterator{ &element_infos_, begins_at_level_[level] };
		}
	}

	const_iterator cbegin_of_level(int level) const {
		CHECK(level >= 0 && level < depth_);
		if (begins_at_level_[level] < 0) {
			return const_iterator{ &element_infos_ };
		}
		else {
			return const_iterator{ &element_infos_, begins_at_level_[level] };
		}
	}

	iterator end_of_level(int level) {
		CHECK(level >= 0 && level < depth_);
		if (prev_ends_at_level_[level] != -1) {
			return iterator{ &element_infos_, element_infos_[prev_ends_at_level_[level]].next };
		}
		else {
			return iterator{ &element_infos_ };
		}
	}

	const_iterator cend_of_level(int level) const {
		CHECK(level >= 0 && level < depth_);
		if (prev_ends_at_level_[level] != -1) {
			return const_iterator{ &element_infos_, element_infos_[prev_ends_at_level_[level]].next };
		}
		else {
			return const_iterator{ &element_infos_ };
		}
	}

	// Reserve space for the specified number of elements.
	void reserve(int n) {
		// max node number is smaller equal to:
		// n * (depth - 1) + (4^1 - 1)/3
		// n * (depth -2) + (4^2 - 1)/3
		// n * (depth - 3) + (4^3 -1)/3
		// ...
		// 0 + (4^depth  -1)/ 3
		// 
		n = std::min(n, max_num_elements_);

		int tmp = static_cast<int> (std::log(n) / std::log(4));
		tmp = std::clamp(tmp, 0, depth_ - 1);

		size_t node_vector_size = n * (depth_ - 1 - tmp) + (2 << ((1 + tmp) * 2)) + 1;
		size_t element_vector_size = n + 2;

		nodes_.reserve(node_vector_size);
		element_infos_.reserve(element_vector_size);
	}


	// Pre-condition: depth() == other_tree.depth().
	// false : merge action abandoned due to capacity concern.
	bool Merge(const QuadTree<T>& other_tree,
		const std::function<void(T*, const T&)>& conflict_resolver) {
		CHECK_EQ(depth_, other_tree.depth_);

		// Although not every new element is added,
		// still return false here to avoid half-way embarrassment.
		if (other_tree.size() > max_num_elements_ - size()) {
			return false;
		}

		// iterate thru other tree's elements. by iterator.
		for (const_iterator iter = other_tree.cbegin(); iter != other_tree.cend(); ++iter) {
			int level, xid, yid;
			std::tie(level, xid, yid) = iter.GetIndices();

			T* self_ele = GetMutableElement(level, xid, yid);
			if (self_ele != nullptr) {
				conflict_resolver(self_ele, *iter);
			}
			else {
				CHECK(AddElement(level, xid, yid, *iter));
			}
		}
		return true;
	}

	// Will clear the tree. 
	void clear() {
		// depth unchanged;
		// Reserved memory unchanged.
		nodes_.clear();
		element_infos_.clear();
		root_ = 0;
		begins_at_level_.clear();
		prev_ends_at_level_.clear();
		num_elements_at_level_.clear();
		num_elements_ = 0;

		// Re-do initializer.
		num_elements_at_level_.resize(depth_, 0);
		nodes_.emplace_back(); // The "nullptr"
		element_infos_.emplace_back(); // Head
		element_infos_.emplace_back(); // Tail
		element_infos_[kHead].next = kTail;
		element_infos_[kTail].prev = kHead;

		begins_at_level_.resize(depth_, -1);
		prev_ends_at_level_.resize(depth_, -1);
	};

	const int max_num_elements() const {
		return max_num_elements_;
	}


private:
	static constexpr int kHead = 0;
	static constexpr int kTail = 1;
	const int max_num_elements_ = INT_MAX - 2;
	int DeleteChildLessNode(int id) {
		const int last_id = static_cast<int>(nodes_.size()) - 1;
		int father = 0;
		if (id > 0 && id < nodes_.size()) {
			// Step 1. make sure every adjacent nodes know "id" is dead.
			const Node& current = nodes_[id];
			father = current.father;
			if (current.father != 0) {
				nodes_[current.father].children[current.id_as_a_child] = 0;
			}
			if (root_ == id) {
				root_ = 0;
			}
			// No child notificationn needed.
			// No element notification needed.

			// Step 2, if "id" is not the last element.
			// Move last element to the slot at id.
			if (last_id != id) {
				const Node& last = nodes_[last_id];
				if (last.father != 0) {
					nodes_[last.father].children[last.id_as_a_child] = id;
				}
				for (int i = 0; i < 4; ++i) {
					if (last.children[i] != 0) {
						nodes_[last.children[i]].father = id;
					}
				}
				if (last.element_info_id >= 0) {
					element_infos_[last.element_info_id].node_id = id;
				}
				if (root_ == last_id) {
					root_ = id;
				}
				if (father == last_id) {
					father = id;
				}
				nodes_[id] = nodes_[last_id];
			}

			// Step 3. pop last element.
			nodes_.pop_back();
		}
		return father;
	}



	int ElementInfoIdFromIndices(int level, int xid, int yid) const {
		CHECK(IsInside(level, xid, yid));

		int node = root_;
		int mid = 0;
		int id_as_a_child = 0;
		// Avoid recursive calling, use while instead.
		while (level > 0 && node != 0) {
			mid = 1 << level - 1;
			if (xid >= mid) {
				xid -= mid;
				if (yid >= mid) {
					id_as_a_child = 3;
					yid -= mid;
				}
				else {
					id_as_a_child = 2;
				}
			}
			else {
				if (yid >= mid) {
					id_as_a_child = 1;
					yid -= mid;
				}
				else {
					id_as_a_child = 0;
				}
			}
			node = nodes_[node].children[id_as_a_child];
			level--;
		}

		if (node == 0) {
			return -1;
		}

		return nodes_[node].element_info_id;
	}

	int ElementInfoIdFromPosition(double x, double y, int max_level, double root_box_width = 1.) const {
		if (!IsPositionInside(x, y, root_box_width)) {
			return -1;
		}
		max_level = std::min(max_level, depth_ - 1);
		if (max_level < 0) {
			return -1;
		}

		int node = root_;
		int target_element_info = -1;
		int curr_level = 0;
		double half_width = root_box_width * 0.5;

		// Avoid recursive calling, use while instead.
		while (curr_level < max_level && node != 0) {
			if (nodes_[node].element_info_id >= 0) {
				target_element_info = nodes_[node].element_info_id;
			}

			int id_as_a_child = 0;
			if (x >= half_width) {
				x -= half_width;
				if (y >= half_width) {
					id_as_a_child = 3;
					y -= half_width;
				}
				else {
					id_as_a_child = 2;
				}
			}
			else {
				if (y >= half_width) {
					id_as_a_child = 1;
					y -= half_width;
				}
				else {
					id_as_a_child = 0;
				}
			}
			node = nodes_[node].children[id_as_a_child];
			curr_level++;
			half_width *= 0.5;
		}

		if (node != 0 && nodes_[node].element_info_id >= 0) {
			target_element_info = nodes_[node].element_info_id;
		}

		return target_element_info;
	}

	ElementInfoIndexArray ElementInfoIdsFromPosition(double x, double y, int max_level, double root_box_width = 1.) const {
		ElementInfoIndexArray result;

		if (!IsPositionInside(x, y, root_box_width)) {
			return result;
		}
		max_level = std::min(max_level, depth_ - 1);
		if (max_level < 0) {
			return result;
		}

		int node = root_;
		int curr_level = 0;
		double half_width = root_box_width * 0.5;

		// Avoid recursive calling, use while instead.
		while (curr_level < max_level && node != 0) {
			int element_id = nodes_[node].element_info_id;
			if (element_id >= 0) {
				result.elements[result.num_elements] = element_id;
				result.num_elements += 1;
			}

			int id_as_a_child = 0;

			if (x >= half_width) {
				x -= half_width;
				if (y >= half_width) {
					id_as_a_child = 3;
					y -= half_width;
				}
				else {
					id_as_a_child = 2;
				}
			}
			else {
				if (y >= half_width) {
					id_as_a_child = 1;
					y -= half_width;
				}
				else {
					id_as_a_child = 0;
				}
			}
			node = nodes_[node].children[id_as_a_child];
			curr_level++;
			half_width *= 0.5;
		}

		if (node != 0 && nodes_[node].element_info_id >= 0) {
			result.elements[result.num_elements] = nodes_[node].element_info_id;
			result.num_elements += 1;
		}
		return result;
	}



	int depth_ = 0;
	double max_id_upper_bound_inv_ = 1.0;

	std::vector<Node> nodes_;
	std::vector<ElementInfo> element_infos_;

	int root_ = 0;
	std::vector<int> begins_at_level_;
	std::vector<int> prev_ends_at_level_;

	std::vector<size_t> num_elements_at_level_;
	size_t num_elements_ = 0;
};
