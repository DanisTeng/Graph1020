#pragma once


#include "quad_tree.h"
#include "grid_frame.h"
#include "geometry.h"
#include "grid_set.h"

// MOI: multi-object interaction
// There are many objects, each will interact with all others.
// Time complexity is too high.
// When only those "spacially close ranged" can interact with the target object. 
// It is possible to avoid enumerating them all.

// An object affects another one by first of all claim it impact in certain spacial domain,
// known as "field", the other object will then get affected by the field.

// Multiple object's field will don't have to be necessarily adding up.
// But there is always a way to tell their joint impact to the space.
// To tell the final field. etc.


// 1, Field exists in space, for limited space, field can be expressed/calculated in simpler way
// 2, Each field has limited spacial impact.


namespace close_range_moi {
	// ���������� �� �� ��
	class ObjectFieldContainer {
	public:
		class Field {
			virtual bool CanNotAffect(const geom::Vec2d& box_center, double box_width) = 0; // redundant.
			virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;
			virtual double Complexity() const;
		};

		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const Field& field, 
			const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;

		// How a field can be simplified
		virtual Field CalculateSuperPosedField(const std::vector<Field>& fields, const geom::Vec2d& box_center, double box_width) = 0;
	};

	class LimitedSpaceField {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;
		virtual double Complexity() const = 0;
		virtual

		virtual FromFieldSuperPosition(const std::vector<LimitedSpaceField*>& fields, const geom::Vec2d& box_center, double box_width) = 0;
	};

	// ���Ķ���� ����š��һ���ˡ� ��Ϊ����ʵ��ȡ���ڶ��塣
	// ��������Ҫ�������ģ� �����������ķ����� affected grid���⣬ ��һ��Ҫ���������ӵĹ��ܡ�
	// ���ӹ����ǻ������������Ի���������������ʱ����ͬʱȷ�����г������͵ġ�

	// ����ÿһ�ֳ������Ͳ�һ���� ����Ľӿ�����ֻ���ǳ���ģ� ��������Ҳ��ˡ�

	// ��������£� ��Ҫ֪��ÿ������ϸ��ʵ�֣� û���ó���Ľӿ������� ������������ ʱ�� 
	// ��������Ҫ�����֪���ӿ�������ʲô��  ��������ӿڵ�����Ҫ������ʱͬ��ȷ����
	// ����C++ �����˵ģ� ������� ������ʱͬʱȷ���ӿڡ� ��������û�еġ�

	// �ӿ�ֻ����ȷ������. ������ָ�롣 ָ���ʹ��ҲҪ��������Ϊ��ȷ����

	// ��Ҳ���� �����Ͷ��� ��ϵ�ԭ��
	class Field {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;

	};


	class LimitedSpaceFieldContainer {

	};

	using FieldId = int;
	class LimitedSpaceFieldManager {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(FieldId fid, const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;
		virtual double Complexity(FieldId fid) const = 0;

		// Always assume the superposition is simpler than a vector of fields.
		virtual int FieldSuperPosition(const std::vector<FieldId>& fids, const geom::Vec2d& box_center, double box_width) = 0;
	};
	
	class LimitedSpaceFieldMap {
		// Only handle index
		// input: vector of FieldId as initial fields
		// output: me,  can accept querry in vec2d, lead you to a field very fast.
	};



	// The quad tree will 


	class QuadTreeMOI2d {
	public:



		class Object {
			virtual bool CanNotAffect(const geom::Vec2d& box_center, double box_width) = 0;
			virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;
		};
		




	};


}  // namespace close_range_space


