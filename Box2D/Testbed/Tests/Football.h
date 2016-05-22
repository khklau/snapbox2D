#ifndef FOOTBALL_H
#define FOOTBALL_H

#include <array>
#include <limits>
#include "../Framework/Test.h"
#include "../Box2D/Box2D/Dynamics/b2World.h"
#include "../Box2D/Box2D/Collision/Shapes/b2CircleShape.h"

static const float deg2rad = 0.0174532925199432957f;
static const float rad2deg = 57.295779513082320876f;

namespace entity
{
	enum type : std::uint16_t
	{
		ball = 0,
		goal,
		goal_sensor,
		player_torso,
		player_foot,
		player_head,
		player_sensor,
		field_sensor
	};
}

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
	fix_def.filter.categoryBits = entity::ball;
	fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
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
	torso(b2World& world, std::uint8_t id);
	~torso() noexcept;
	inline b2Body& get_body(const torso_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

torso::torso(b2World& world, std::uint8_t id)
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
	fix_def.filter.categoryBits = entity::player_torso;
	fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	fix_def.filter.groupIndex = id * -1;
	fix_def.density = 4.0f;
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);

	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
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
	head(b2World& world, std::uint8_t id, std::size_t vision_radius = 20U, std::size_t vision_degree = 120U);
	~head() noexcept;
	inline b2Body& get_body(const head_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* head_fixture_;
	b2Fixture* vision1_fixture_;
	b2Fixture* vision2_fixture_;
};

head::head(b2World& world, std::uint8_t id, std::size_t vision_radius, std::size_t vision_degree)
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
	head_fix_def.filter.categoryBits = entity::player_head;
	head_fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	head_fix_def.filter.groupIndex = id * -1;
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
	vision1_fix_def.density = 0;
	vision1_fix_def.isSensor = true;
	vision1_fix_def.filter.categoryBits = entity::player_sensor;
	vision1_fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	vision1_fix_def.filter.groupIndex = id * -1;
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
	vision2_fix_def.density = 0;
	vision2_fix_def.isSensor = true;
	vision2_fix_def.filter.categoryBits = entity::player_sensor;
	vision2_fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	vision2_fix_def.filter.groupIndex = id * -1;
	vision2_fixture_ = body_->CreateFixture(&vision2_fix_def);

	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
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
	foot(b2World& world, std::uint8_t id);
	~foot() noexcept;
	inline b2Body& get_body(const foot_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

foot::foot(b2World& world, std::uint8_t id)
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
	fix_def.filter.categoryBits = entity::player_foot;
	fix_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	fix_def.filter.groupIndex = id * -1;
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);
	
	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
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
	void turn(std::int16_t velocity);
	inline float get_head_angle() const { return joint_->GetBodyB()->GetAngle(); };
	inline float get_head_velocity() const { return joint_->GetBodyB()->GetAngularVelocity(); }
private:
	inline b2Body& get_torso_body() { return *(joint_->GetBodyA()); }
	inline b2Body& get_head_body() { return *(joint_->GetBodyB()); }
	b2World& world_;
	b2RevoluteJoint* joint_;
	std::int16_t velocity_;
};

neck::neck(b2World& world, torso& torso, head& head)
	:
		world_(world),
		joint_(nullptr),
		velocity_(0)
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

void neck::turn(std::int16_t velocity)
{
	velocity_ = velocity;
	get_head_body().ApplyAngularImpulse(velocity_ / 1000.0, true);
}

class hip
{
public:
	hip(b2World& world, torso& torso, foot& foot);
	~hip() noexcept;
	void move(std::int16_t velocity);
	void turn(std::int16_t velocity);
	inline float get_torso_angle() const { return joint_->GetBodyA()->GetAngle(); }
	inline float get_torso_angular_velocity() const { return joint_->GetBodyA()->GetAngularVelocity(); }
	inline float get_foot_translation() const { return joint_->GetJointTranslation(); }
	inline float get_foot_speed() const { return joint_->GetJointSpeed(); }
private:
	inline b2Body& get_torso_body() { return *(joint_->GetBodyA()); }
	inline b2Body& get_foot_body() { return *(joint_->GetBodyB()); }
	b2World& world_;
	b2PrismaticJoint* joint_;
	std::int16_t torso_angular_velocity_;
	std::int16_t foot_velocity_;
};

hip::hip(b2World& world, torso& torso, foot& foot)
	:
		world_(world),
		joint_(nullptr),
		torso_angular_velocity_(0),
		foot_velocity_(0)
{
	b2PrismaticJointDef joint_def;
	joint_def.bodyA = &torso.get_body(torso_key());
	joint_def.bodyB = &foot.get_body(foot_key());
	joint_def.localAnchorA.Set(0, 0);
	joint_def.localAnchorB.Set(0, 0);
	joint_def.localAxisA.Set(1, 0);
	joint_def.localAxisA.Normalize();
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

void hip::move(std::int16_t velocity)
{
	foot_velocity_ = velocity;
	b2Vec2 target(0, 0);
	target.x = foot_velocity_ / 1000.0 * cosf(get_torso_angle());
	target.y = foot_velocity_ / 1000.0 * sinf(get_torso_angle());
	get_foot_body().ApplyLinearImpulse(target, get_foot_body().GetWorldCenter(), true);
}

void hip::turn(std::int16_t velocity)
{
	torso_angular_velocity_ = velocity;
	get_torso_body().ApplyAngularImpulse(torso_angular_velocity_ / 1000.0, true);
	get_foot_body().ApplyAngularImpulse(torso_angular_velocity_ / 1000.0, true);
}

class player
{
public:
	player(b2World& world, std::uint8_t id);
	inline void turn_head(std::int16_t velocity) { neck_.turn(velocity); }
	inline void turn_torso(std::int16_t velocity) { hip_.turn(velocity); }
	inline void move_foot(std::int16_t velocity) { hip_.move(velocity); }
	inline float get_head_angle() const { return neck_.get_head_angle(); }
	inline float get_head_velocity() const { return neck_.get_head_velocity(); }
	inline float get_torso_angle() const { return hip_.get_torso_angle(); }
	inline float get_torso_angular_velocity() const { return hip_.get_torso_angular_velocity(); }
	inline float get_foot_translation() const { return hip_.get_foot_translation(); }
	inline float get_foot_speed() const { return hip_.get_foot_speed(); }
private:
	std::uint8_t id_;
	torso torso_;
	head head_;
	foot foot_;
	neck neck_;
	hip hip_;
};

player::player(b2World& world, std::uint8_t id)
	:
		id_(id),
		torso_(world, id_),
		head_(world, id_),
		foot_(world, id_),
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
	back_def.filter.categoryBits = entity::goal;
	back_def.filter.maskBits = std::numeric_limits<entity::type>::max();
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
	left_net_def.filter.categoryBits = entity::goal;
	left_net_def.filter.maskBits = std::numeric_limits<entity::type>::max();
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
	right_net_def.filter.categoryBits = entity::goal;
	right_net_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	right_net_ = body_->CreateFixture(&right_net_def);

	b2CircleShape left_post;
	left_post.m_p.Set(0.0, 12.0);
	left_post.m_radius = 0.5;
	b2FixtureDef left_post_def;
	left_post_def.shape = &left_post;
	left_post_def.filter.categoryBits = entity::goal;
	left_post_def.filter.maskBits = std::numeric_limits<entity::type>::max();
	left_post_ = body_->CreateFixture(&left_post_def);

	b2CircleShape right_post;
	right_post.m_p.Set(0.0, -12.0);
	right_post.m_radius = 0.5;
	b2FixtureDef right_post_def;
	right_post_def.shape = &right_post;
	right_post_def.filter.categoryBits = entity::goal;
	right_post_def.filter.maskBits = std::numeric_limits<entity::type>::max();
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
	sensor_def.density = 0;
	sensor_def.isSensor = true;
	sensor_def.filter.categoryBits = entity::goal_sensor;
	sensor_def.filter.maskBits = entity::ball;
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
	virtual void Keyboard(int key);
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
		player1_(*m_world, 1),
		left_goal_(*m_world)
{
	m_world->SetGravity(b2Vec2(0, 0));
}

void Football::Step(Settings* settings)
{
	Test::Step(settings);
	g_debugDraw.DrawString(5, m_textLine, "Player head angle: %3.2f", player1_.get_head_angle() * rad2deg);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player head velocity: %3.2f", player1_.get_head_velocity());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player torso angle: %3.2f", player1_.get_torso_angle() * rad2deg);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player torso angular velocity: %3.2f", player1_.get_torso_angular_velocity());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player foot translation: %3.2f", player1_.get_foot_translation());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player foot speed: %3.2f", player1_.get_foot_speed());
	m_textLine += DRAW_STRING_NEW_LINE;
}

void Football::Keyboard(int key)
{
	switch (key)
	{
		case GLFW_KEY_Q:
			player1_.turn_head(200);
			break;
		case GLFW_KEY_E:
			player1_.turn_head(-200);
			break;
		case GLFW_KEY_A:
			player1_.turn_torso(400);
			break;
		case GLFW_KEY_D:
			player1_.turn_torso(-400);
			break;
		case GLFW_KEY_3:
			player1_.move_foot(3000);
			break;
		case GLFW_KEY_1:
			player1_.move_foot(-3000);
			break;
	}
}

Test* Football::Create()
{
	return new Football();
}

#endif