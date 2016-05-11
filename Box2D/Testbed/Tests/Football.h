#ifndef FOOTBALL_H
#define FOOTBALL_H

#include <array>
#include "../Framework/Test.h"
#include "../Box2D/Box2D/Dynamics/b2World.h"
#include "../Box2D/Box2D/Collision/Shapes/b2CircleShape.h"

static const float deg2rad = 0.0174532925199432957f;
static const float rad2deg = 57.295779513082320876f;

class ball
{
public:
	explicit ball(b2World& world);
	~ball() noexcept;
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

ball::ball(b2World& world)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(0, 20);
	body_def.angle = 0 * deg2rad;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = 1;
	b2FixtureDef fix_def;
	fix_def.shape = &circle;
	fixture_ = body_->CreateFixture(&fix_def);
}

ball::~ball() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

struct torso_key
{
	friend class neck;
	friend class hip;
};

class torso
{
public:
	explicit torso(b2World& world);
	~torso() noexcept;
	inline b2Body& get_body(const torso_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

torso::torso(b2World& world)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(10, 10);
	body_def.angle = 0 * deg2rad;
	body_ = world_.CreateBody(&body_def);

	std::array<b2Vec2, 8> vertices;
	vertices[0].Set(1, -1);
	vertices[1].Set(1, 1);
	vertices[2].Set(0.5, 2);
	vertices[3].Set(-0.5, 2);
	vertices[4].Set(-1, 1);
	vertices[5].Set(-1, -1);
	vertices[6].Set(-0.5, -2);
	vertices[7].Set(0.5, -2);
	b2PolygonShape poly;
	poly.Set(vertices.data(), vertices.max_size());
	b2FixtureDef fix_def;
	fix_def.shape = &poly;
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);
}

torso::~torso() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

struct head_key
{
	friend class neck;
};

class head
{
public:
	head(b2World& world, std::size_t vision_radius = 20U, std::size_t vision_degree = 120U);
	~head() noexcept;
	inline b2Body& get_body(const head_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* head_fixture_;
	b2Fixture* vision1_fixture_;
	b2Fixture* vision2_fixture_;
};

head::head(b2World& world, std::size_t vision_radius, std::size_t vision_degree)
	:
		world_(world),
		body_(nullptr),
		head_fixture_(nullptr),
		vision1_fixture_(nullptr),
		vision2_fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(0, 0);
	body_def.angle = 0 * deg2rad;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = 1;
	b2FixtureDef head_fix_def;
	head_fix_def.shape = &circle;
	head_fixture_ = body_->CreateFixture(&head_fix_def);
	
	std::size_t arc_degree = vision_degree / 2;

	b2PolygonShape arc1;
	std::array<b2Vec2, 8> vertices1;
	vertices1[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices1.max_size() - 1); ++vertex)
	{
		float angle = ((vertex / (vertices1.max_size() - 2.0) * arc_degree) - arc_degree) *deg2rad;
		vertices1[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc1.Set(vertices1.data(), vertices1.max_size());
	b2FixtureDef vision1_fix_def;
	vision1_fix_def.shape = &arc1;
	vision1_fixture_ = body_->CreateFixture(&vision1_fix_def);

	b2PolygonShape arc2;
	std::array<b2Vec2, 8> vertices2;
	vertices2[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices2.max_size() - 1); ++vertex)
	{
		float angle = (vertex / (vertices1.max_size() - 2.0) * arc_degree) *deg2rad;
		vertices2[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc2.Set(vertices2.data(), vertices2.max_size());
	b2FixtureDef vision2_fix_def;
	vision2_fix_def.shape = &arc2;
	vision2_fixture_ = body_->CreateFixture(&vision2_fix_def);
}

head::~head() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

struct foot_key
{
	friend class hip;
};

class foot
{
public:
	explicit foot(b2World& world);
	~foot() noexcept;
	inline b2Body& get_body(const foot_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

foot::foot(b2World& world)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(10, 10);
	body_def.angle = 0 * deg2rad;
	body_ = world_.CreateBody(&body_def);

	std::array<b2Vec2, 4> vertices;
	vertices[0].Set(1, 0.5);
	vertices[1].Set(1, -0.5);
	vertices[2].Set(-1, -0.5);
	vertices[3].Set(-1, 0.5);
	b2PolygonShape poly;
	poly.Set(vertices.data(), vertices.max_size());
	b2FixtureDef fix_def;
	fix_def.shape = &poly;
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);
}

foot::~foot() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

class neck
{
public:
	neck(b2World& world, torso& torso, head& head);
	~neck() noexcept;
private:
	b2World& world_;
	b2RevoluteJoint* joint_;
};

neck::neck(b2World& world, torso& torso, head& head)
	:
		world_(world),
		joint_(nullptr)
{
	b2RevoluteJointDef joint_def;
	joint_def.bodyA = &torso.get_body(torso_key());
	joint_def.bodyB = &head.get_body(head_key());
	joint_def.collideConnected = false;
	joint_def.localAnchorA.Set(0, 0);
	joint_def.localAnchorB.Set(0, 0);
	joint_def.referenceAngle = 0;
	joint_def.enableLimit = true;
	joint_def.lowerAngle = -60 * deg2rad;
	joint_def.upperAngle = 60 * deg2rad;
	joint_ = static_cast<b2RevoluteJoint*>(world_.CreateJoint(&joint_def));
}

neck::~neck() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

class hip
{
public:
	hip(b2World& world, torso& torso, foot& foot);
	~hip() noexcept;
private:
	b2World& world_;
	b2PrismaticJoint* joint_;
};

hip::hip(b2World& world, torso& torso, foot& foot)
	:
		world_(world),
		joint_(nullptr)
{
	b2PrismaticJointDef joint_def;
	joint_def.bodyA = &torso.get_body(torso_key());
	joint_def.bodyB = &foot.get_body(foot_key());
	joint_def.localAnchorA.Set(0, 0);
	joint_def.localAnchorB.Set(0, 0);
	joint_def.localAxisA.Set(0, 1);
	joint_def.enableLimit = true;
	joint_def.lowerTranslation = 0;
	joint_def.upperTranslation = 2;
	joint_ = static_cast<b2PrismaticJoint*>(world_.CreateJoint(&joint_def));
}

hip::~hip() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

class player
{
public:
	explicit player(b2World& world);
private:
	torso torso_;
	head head_;
	foot foot_;
	neck neck_;
	hip hip_;
};

player::player(b2World& world)
	:
		torso_(world),
		head_(world),
		foot_(world),
		neck_(world, torso_, head_),
		hip_(world, torso_, foot_)
{}

class goal
{
public:
	explicit goal(b2World& world);
	~goal() noexcept;
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* back_net_;
	b2Fixture* left_net_;
	b2Fixture* right_net_;
	b2Fixture* left_post_;
	b2Fixture* right_post_;
	b2Fixture* sensor_;
};

goal::goal(b2World& world)
	:
		world_(world),
		body_(nullptr),
		back_net_(nullptr),
		left_net_(nullptr),
		right_net_(nullptr),
		left_post_(nullptr),
		right_post_(nullptr),
		sensor_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_staticBody;
	body_def.position.Set(-20, 20);
	body_def.angle = 0;
	body_ = world_.CreateBody(&body_def);

	std::array<b2Vec2, 2> back_vertices;
	back_vertices[0].Set(-4.0, 12.0);
	back_vertices[1].Set(-4.0, -12.0);
	b2EdgeShape back_shape;
	back_shape.Set(back_vertices[0], back_vertices[1]);
	back_shape.m_hasVertex0 = false;
	back_shape.m_hasVertex3 = false;
	b2FixtureDef back_def;
	back_def.shape = &back_shape;
	back_net_ = body_->CreateFixture(&back_def);

	std::array<b2Vec2, 2> left_vertices;
	left_vertices[0].Set(-4.0, 12.0);
	left_vertices[1].Set(-0.5, 12.0);
	b2EdgeShape left_shape;
	left_shape.Set(left_vertices[0], left_vertices[1]);
	left_shape.m_hasVertex0 = false;
	left_shape.m_hasVertex3 = false;
	b2FixtureDef left_net_def;
	left_net_def.shape = &left_shape;
	left_net_ = body_->CreateFixture(&left_net_def);

	std::array<b2Vec2, 2> right_vertices;
	right_vertices[0].Set(-4.0, -12.0);
	right_vertices[1].Set(-0.5, -12.0);
	b2EdgeShape right_shape;
	right_shape.Set(right_vertices[0], right_vertices[1]);
	right_shape.m_hasVertex0 = false;
	right_shape.m_hasVertex3 = false;
	b2FixtureDef right_net_def;
	right_net_def.shape = &right_shape;
	right_net_ = body_->CreateFixture(&right_net_def);

	b2CircleShape left_post;
	left_post.m_p.Set(0.0, 12.0);
	left_post.m_radius = 0.5;
	b2FixtureDef left_post_def;
	left_post_def.shape = &left_post;
	left_post_ = body_->CreateFixture(&left_post_def);

	b2CircleShape right_post;
	right_post.m_p.Set(0.0, -12.0);
	right_post.m_radius = 0.5;
	b2FixtureDef right_post_def;
	right_post_def.shape = &right_post;
	right_post_ = body_->CreateFixture(&right_post_def);

	std::array<b2Vec2, 4> sensor_vertices;
	sensor_vertices[0].Set(-4.0, 12.0);
	sensor_vertices[1].Set(-2.0, 12.0);
	sensor_vertices[2].Set(-2.0, -12.0);
	sensor_vertices[3].Set(-4.0, -12.0);
	b2PolygonShape sensor_box;
	sensor_box.Set(sensor_vertices.data(), sensor_vertices.max_size());
	b2FixtureDef sensor_def;
	sensor_def.shape = &sensor_box;
	sensor_ = body_->CreateFixture(&sensor_def);
}

goal::~goal() noexcept
{
	try
	{
	}
	catch (...)
	{
		// do nothing
	}
}

class Football : public Test
{
public:
	Football();
	virtual void Step(Settings* settings);
	static Test* Create();
private:
	ball ball_;
	player player1_;
	goal left_goal_;
};

Football::Football()
	:
		Test(),
		ball_(*m_world),
		player1_(*m_world),
		left_goal_(*m_world)
{
	m_world->SetGravity(b2Vec2(0, 0));
}

void Football::Step(Settings* settings)
{
	Test::Step(settings);
}

Test* Football::Create()
{
	return new Football();
}

#endif