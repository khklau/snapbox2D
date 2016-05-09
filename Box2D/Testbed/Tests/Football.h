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
	body_def.position.Set(20, 20);
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
	vertices[0].Set(-1, -1);
	vertices[1].Set(1, -1);
	vertices[2].Set(2, -0.5);
	vertices[3].Set(2, 0.5);
	vertices[4].Set(1, 1);
	vertices[5].Set(-1, 1);
	vertices[6].Set(-2, 0.5);
	vertices[7].Set(-2, -0.5);
	b2PolygonShape poly;
	poly.Set(vertices.data(), vertices.size());
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
	explicit head(b2World& world);
	~head() noexcept;
	inline b2Body& get_body(const head_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

head::head(b2World& world)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(0, 0);
	body_def.angle = 90 * deg2rad;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = 1;
	b2FixtureDef fix_def;
	fix_def.shape = &circle;
	fixture_ = body_->CreateFixture(&fix_def);
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
	vertices[0].Set(-0.5, -1);
	vertices[1].Set(0.5, -1);
	vertices[2].Set(0.5, 1);
	vertices[3].Set(-0.5, 1);
	b2PolygonShape poly;
	poly.Set(vertices.data(), vertices.size());
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
	~neck();
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
	~hip();
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

class Football : public Test
{
public:
	Football();
	virtual void Step(Settings* settings);
	static Test* Create();
private:
	ball ball_;
	player player1_;
};

Football::Football()
	:
		Test(),
		ball_(*m_world),
		player1_(*m_world)
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