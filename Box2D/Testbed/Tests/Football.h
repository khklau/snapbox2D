#ifndef FOOTBALL_H
#define FOOTBALL_H

#include <array>
#include <limits>
#include <type_traits>
#include <stdexcept>
#include "../Framework/Test.h"
#include "../Box2D/Box2D/Dynamics/b2World.h"
#include "../Box2D/Box2D/Collision/Shapes/b2CircleShape.h"
#include "../Box2D/Box2D/Dynamics/b2WorldCallbacks.h"

static const float deg2rad = 0.0174532925199432957f;
static const float rad2deg = 57.295779513082320876f;

namespace entity
{
	enum type : std::uint16_t
	{
		ball = 2,
		goal_left_post,
		goal_right_post,
		goal_net,
		goal_sensor,
		player_torso,
		player_foot,
		player_head,
		player_sensor,
		center_circle,
		boundary,
		boundary_sensor,
		left_penalty_box,
		right_penalty_box
	};
	enum id : std::uint16_t
	{
		alpha_1 = 1,
		alpha_2,
		alpha_3,
		alpha_4,
		alpha_5,
		alpha_6,
		alpha_7,
		alpha_8,
		alpha_9,
		alpha_10,
		alpha_11,
		beta_1,
		beta_2,
		beta_3,
		beta_4,
		beta_5,
		beta_6,
		beta_7,
		beta_8,
		beta_9,
		beta_10,
		beta_11,
		the_ball,
		left_goal,
		right_goal,
		spot,
		top,
		bottom,
		left,
		right,
		top_left,
		top_right,
		bottom_left,
		bottom_right
	};
	inline decltype(b2Filter::categoryBits) type2category(entity::type type) { return 1 << (type - 1U); }
	entity::type category2type(decltype(b2Filter::categoryBits) category);
	void config_fixture(b2FixtureDef& def, const type& type, const id& id);
	inline type get_type(const b2Fixture& fixture) { return category2type(fixture.GetFilterData().categoryBits); }
	inline id get_id(const b2Fixture& fixture) { return static_cast<id>(fixture.GetFilterData().groupIndex * -1); }
}

entity::type entity::category2type(decltype(b2Filter::categoryBits) category)
{
	switch (category)
	{
		case 2U:
			return static_cast<entity::type>(1U + 1U);
		case 4U:
			return static_cast<entity::type>(2U + 1U);
		case 8U:
			return static_cast<entity::type>(3U + 1U);
		case 16U:
			return static_cast<entity::type>(4U + 1U);
		case 32U:
			return static_cast<entity::type>(5U + 1U);
		case 64U:
			return static_cast<entity::type>(6U + 1U);
		case 128U:
			return static_cast<entity::type>(7U + 1U);
		case 256U:
			return static_cast<entity::type>(8U + 1U);
		case 512U:
			return static_cast<entity::type>(9U + 1U);
		case 1024U:
			return static_cast<entity::type>(10U + 1U);
		case 2048U:
			return static_cast<entity::type>(11U + 1U);
		case 4096U:
			return static_cast<entity::type>(12U + 1U);
		case 8192U:
			return static_cast<entity::type>(13U + 1U);
		case 16384U:
			return static_cast<entity::type>(14U + 1U);
		case 32768U:
			return static_cast<entity::type>(15U + 1U);
		default:
			throw std::out_of_range("Not within entity::type range");
	}
}

void entity::config_fixture(b2FixtureDef& def, const type& type, const id& id)
{
	def.filter.categoryBits = type2category(type);
	def.filter.groupIndex = id * -1;
}

class ball
{
public:
	ball(b2World& world, const b2Vec2& position);
	~ball() noexcept;
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

ball::ball(b2World& world, const b2Vec2& position)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(position.x, position.y);
	body_def.angle = 0 * deg2rad;
	body_def.linearDamping = 0.15f;
	body_def.angularDamping = 0.15f;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = 1;
	b2FixtureDef fix_def;
	fix_def.shape = &circle;
	entity::config_fixture(fix_def, entity::ball, entity::the_ball);
	fix_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	fixture_ = body_->CreateFixture(&fix_def);
}

ball::~ball() noexcept
{
	try
	{
		world_.DestroyBody(body_);
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
	torso(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree);
	~torso() noexcept;
	inline b2Body& get_body(const torso_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

torso::torso(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.linearDamping = 0.15f;
	body_def.angularDamping = 0.15f;
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
	entity::config_fixture(fix_def, entity::player_torso, id);
	fix_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	fix_def.density = 4.0f;
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);

	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
	body_->SetTransform(position, degree * deg2rad);
}

torso::~torso() noexcept
{
	try
	{
		world_.DestroyBody(body_);
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
	head(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree, std::size_t vision_radius = 60U, std::size_t vision_degree = 120U);
	~head() noexcept;
	inline b2Body& get_body(const head_key&) { return *body_; }
private:
	static decltype(b2Filter::maskBits) not_sensor_mask();
	b2World& world_;
	b2Body* body_;
	b2Fixture* head_fixture_;
	b2Fixture* vision1_fixture_;
	b2Fixture* vision2_fixture_;
	b2Fixture* vision3_fixture_;
	b2Fixture* vision4_fixture_;
};

head::head(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree, std::size_t vision_radius, std::size_t vision_degree)
	:
		world_(world),
		body_(nullptr),
		head_fixture_(nullptr),
		vision1_fixture_(nullptr),
		vision2_fixture_(nullptr),
		vision3_fixture_(nullptr),
		vision4_fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.linearDamping = 0.15f;
	body_def.angularDamping = 0.15f;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = 1;
	b2FixtureDef head_fix_def;
	entity::config_fixture(head_fix_def, entity::player_head, id);
	head_fix_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	head_fix_def.shape = &circle;

	head_fixture_ = body_->CreateFixture(&head_fix_def);
	
	std::size_t arc_degree = vision_degree / 4;

	b2PolygonShape arc1;
	std::array<b2Vec2, 8> vertices1;
	vertices1[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices1.max_size() - 1); ++vertex)
	{
		float angle = ((vertex / (vertices1.max_size() - 2.0) * arc_degree) - (arc_degree * 2)) *deg2rad;
		vertices1[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc1.Set(vertices1.data(), vertices1.max_size());
	b2FixtureDef vision1_fix_def;
	vision1_fix_def.shape = &arc1;
	vision1_fix_def.density = 0;
	vision1_fix_def.isSensor = true;
	entity::config_fixture(vision1_fix_def, entity::player_sensor, id);
	vision1_fix_def.filter.maskBits = not_sensor_mask();
	vision1_fixture_ = body_->CreateFixture(&vision1_fix_def);

	b2PolygonShape arc2;
	std::array<b2Vec2, 8> vertices2;
	vertices2[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices2.max_size() - 1); ++vertex)
	{
		float angle = ((vertex / (vertices1.max_size() - 2.0) * arc_degree) - arc_degree) *deg2rad;
		vertices2[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc2.Set(vertices2.data(), vertices2.max_size());
	b2FixtureDef vision2_fix_def;
	vision2_fix_def.shape = &arc2;
	vision2_fix_def.density = 0;
	vision2_fix_def.isSensor = true;
	entity::config_fixture(vision2_fix_def, entity::player_sensor, id);
	vision2_fix_def.filter.maskBits = not_sensor_mask();
	vision2_fixture_ = body_->CreateFixture(&vision2_fix_def);
	
	b2PolygonShape arc3;
	std::array<b2Vec2, 8> vertices3;
	vertices3[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices3.max_size() - 1); ++vertex)
	{
		float angle = ((vertex / (vertices3.max_size() - 2.0) * arc_degree)) *deg2rad;
		vertices3[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc3.Set(vertices3.data(), vertices3.max_size());
	b2FixtureDef vision3_fix_def;
	vision3_fix_def.shape = &arc3;
	vision3_fix_def.density = 0;
	vision3_fix_def.isSensor = true;
	entity::config_fixture(vision3_fix_def, entity::player_sensor, id);
	vision3_fix_def.filter.maskBits = not_sensor_mask();
	vision3_fixture_ = body_->CreateFixture(&vision3_fix_def);

	b2PolygonShape arc4;
	std::array<b2Vec2, 8> vertices4;
	vertices4[0].Set(0, 0);
	for (std::size_t vertex = 0; vertex < (vertices4.max_size() - 1); ++vertex)
	{
		float angle = ((vertex / (vertices1.max_size() - 2.0) * arc_degree) + arc_degree) *deg2rad;
		vertices4[vertex + 1].Set(vision_radius * cosf(angle), vision_radius * sinf(angle));
	}
	arc4.Set(vertices4.data(), vertices4.max_size());
	b2FixtureDef vision4_fix_def;
	vision4_fix_def.shape = &arc4;
	vision4_fix_def.density = 0;
	vision4_fix_def.isSensor = true;
	entity::config_fixture(vision4_fix_def, entity::player_sensor, id);
	vision4_fix_def.filter.maskBits = not_sensor_mask();
	vision4_fixture_ = body_->CreateFixture(&vision4_fix_def);

	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
	body_->SetTransform(position, degree * deg2rad);
}

head::~head() noexcept
{
	try
	{
		world_.DestroyBody(body_);
	}
	catch (...)
	{
		// do nothing
	}
}

decltype(b2Filter::maskBits) head::not_sensor_mask()
{
	return std::numeric_limits<std::underlying_type<entity::type>::type>::max()
			& ~entity::type2category(entity::boundary_sensor)
			& ~entity::type2category(entity::goal_sensor)
			& ~entity::type2category(entity::player_sensor);
}

struct foot_key
{
	friend class hip;
};

class foot
{
public:
	foot(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree);
	~foot() noexcept;
	inline b2Body& get_body(const foot_key&) { return *body_; }
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

foot::foot(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.linearDamping = 0.15f;
	body_def.angularDamping = 0.15f;
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
	entity::config_fixture(fix_def, entity::player_foot, id);
	fix_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	body_def.position.Set(0, 0);
	body_ = world_.CreateBody(&body_def);
	fixture_ = body_->CreateFixture(&fix_def);
	
	b2MassData mass;
	body_->GetMassData(&mass);
	mass.I = 1.0f;
	body_->SetMassData(&mass);
	body_->SetTransform(position, degree * deg2rad);
}

foot::~foot() noexcept
{
	try
	{
		world_.DestroyBody(body_);
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
	void run(std::int16_t velocity);
	void turn(std::int16_t velocity);
	inline float get_head_angle() const { return joint_->GetBodyB()->GetAngle(); };
	inline float get_head_velocity() const { return joint_->GetBodyB()->GetAngularVelocity(); }
	inline float get_torso_angle() const { return joint_->GetBodyA()->GetAngle(); };
	inline float get_torso_velocity() const { return joint_->GetBodyA()->GetAngularVelocity(); }
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
		world_.DestroyJoint(joint_);
	}
	catch (...)
	{
		// do nothing
	}
}

void neck::run(std::int16_t velocity)
{
	b2Vec2 target(0, 0);
	target.x = velocity / 1000.0 * cosf(get_torso_angle());
	target.y = velocity / 1000.0 * sinf(get_torso_angle());
	get_head_body().ApplyLinearImpulse(target, get_head_body().GetWorldCenter(), true);
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
	void kick(std::int16_t velocity);
	void run(std::int16_t velocity);
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
		world_.DestroyJoint(joint_);
	}
	catch (...)
	{
		// do nothing
	}
}

void hip::kick(std::int16_t velocity)
{
	foot_velocity_ = velocity;
	b2Vec2 target(0, 0);
	target.x = foot_velocity_ / 1000.0 * cosf(get_torso_angle());
	target.y = foot_velocity_ / 1000.0 * sinf(get_torso_angle());
	get_foot_body().ApplyLinearImpulse(target, get_foot_body().GetWorldCenter(), true);
}

void hip::run(std::int16_t velocity)
{
	std::int16_t final_velocity = (velocity < 0) ? velocity / 25 : velocity;
	b2Vec2 target(0, 0);
	target.x = final_velocity / 1000.0 * cosf(get_torso_angle());
	target.y = final_velocity / 1000.0 * sinf(get_torso_angle());
	get_torso_body().ApplyLinearImpulse(target, get_torso_body().GetWorldCenter(), true);
	if (velocity < 0)
	{
		get_foot_body().ApplyLinearImpulse(target, get_foot_body().GetWorldCenter(), true);
	}
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
	player(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree);
	inline void run(std::int16_t velocity) { neck_.run(velocity); hip_.run(velocity); }
	inline void turn_head(std::int16_t velocity) { neck_.turn(velocity); }
	inline void turn_torso(std::int16_t velocity) { hip_.turn(velocity); }
	inline void kick(std::int16_t velocity) { hip_.kick(velocity); }
	inline float get_head_angle() const { return neck_.get_head_angle(); }
	inline float get_head_velocity() const { return neck_.get_head_velocity(); }
	inline float get_torso_angle() const { return hip_.get_torso_angle(); }
	inline float get_torso_angular_velocity() const { return hip_.get_torso_angular_velocity(); }
	inline float get_foot_translation() const { return hip_.get_foot_translation(); }
	inline float get_foot_speed() const { return hip_.get_foot_speed(); }
private:
	entity::id id_;
	torso torso_;
	head head_;
	foot foot_;
	neck neck_;
	hip hip_;
};

player::player(b2World& world, entity::id id, const b2Vec2& position, const std::uint16_t degree)
	:
		id_(id),
		torso_(world, id_, position, degree),
		head_(world, id_, position, degree),
		foot_(world, id_, position, degree),
		neck_(world, torso_, head_),
		hip_(world, torso_, foot_)
{}

class goal
{
public:
	goal(b2World& world, entity::id id, const b2Vec2& position);
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

goal::goal(b2World& world, entity::id id, const b2Vec2& position)
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
	body_def.position.Set(position.x, position.y);
	body_def.angle = 0;
	body_ = world_.CreateBody(&body_def);

	float orientation = 1.0;
	if (id == entity::right_goal)
	{
		orientation = -1.0;
	}

	std::array<b2Vec2, 2> back_vertices;
	back_vertices[0].Set(-4.0 * orientation, 12.0 * orientation);
	back_vertices[1].Set(-4.0 * orientation, -12.0 * orientation);
	b2EdgeShape back_shape;
	back_shape.Set(back_vertices[0], back_vertices[1]);
	back_shape.m_hasVertex0 = false;
	back_shape.m_hasVertex3 = false;
	b2FixtureDef back_def;
	back_def.shape = &back_shape;
	entity::config_fixture(back_def, entity::goal_net, id);
	back_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	back_net_ = body_->CreateFixture(&back_def);

	std::array<b2Vec2, 2> left_vertices;
	left_vertices[0].Set(-4.0 * orientation, 12.0 * orientation);
	left_vertices[1].Set(-0.5 * orientation, 12.0 * orientation);
	b2EdgeShape left_shape;
	left_shape.Set(left_vertices[0], left_vertices[1]);
	left_shape.m_hasVertex0 = false;
	left_shape.m_hasVertex3 = false;
	b2FixtureDef left_net_def;
	left_net_def.shape = &left_shape;
	entity::config_fixture(left_net_def, entity::goal_net, id);
	left_net_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	left_net_ = body_->CreateFixture(&left_net_def);

	std::array<b2Vec2, 2> right_vertices;
	right_vertices[0].Set(-4.0 * orientation, -12.0 * orientation);
	right_vertices[1].Set(-0.5 * orientation, -12.0 * orientation);
	b2EdgeShape right_shape;
	right_shape.Set(right_vertices[0], right_vertices[1]);
	right_shape.m_hasVertex0 = false;
	right_shape.m_hasVertex3 = false;
	b2FixtureDef right_net_def;
	right_net_def.shape = &right_shape;
	entity::config_fixture(right_net_def, entity::goal_net, id);
	right_net_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	right_net_ = body_->CreateFixture(&right_net_def);

	b2CircleShape left_post;
	left_post.m_p.Set(0.0, 12.0 * orientation);
	left_post.m_radius = 0.5;
	b2FixtureDef left_post_def;
	left_post_def.shape = &left_post;
	entity::config_fixture(left_post_def, entity::goal_left_post, id);
	left_post_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	left_post_ = body_->CreateFixture(&left_post_def);

	b2CircleShape right_post;
	right_post.m_p.Set(0.0, -12.0 * orientation);
	right_post.m_radius = 0.5;
	b2FixtureDef right_post_def;
	right_post_def.shape = &right_post;
	entity::config_fixture(right_post_def, entity::goal_right_post, id);
	right_post_def.filter.maskBits = std::numeric_limits<std::underlying_type<entity::type>::type>::max();
	right_post_ = body_->CreateFixture(&right_post_def);

	std::array<b2Vec2, 4> sensor_vertices;
	sensor_vertices[0].Set(-4.0 * orientation, 12.0 * orientation);
	sensor_vertices[1].Set(-2.0 * orientation, 12.0 * orientation);
	sensor_vertices[2].Set(-2.0 * orientation, -12.0 * orientation);
	sensor_vertices[3].Set(-4.0 * orientation, -12.0 * orientation);
	b2PolygonShape sensor_box;
	sensor_box.Set(sensor_vertices.data(), sensor_vertices.max_size());
	b2FixtureDef sensor_def;
	sensor_def.shape = &sensor_box;
	sensor_def.density = 0;
	sensor_def.isSensor = true;
	entity::config_fixture(sensor_def, entity::goal_sensor, id);
	sensor_def.filter.maskBits = entity::type2category(entity::ball);
	sensor_ = body_->CreateFixture(&sensor_def);
}

goal::~goal() noexcept
{
	try
	{
		world_.DestroyBody(body_);
	}
	catch (...)
	{
		// do nothing
	}
}

class field_sensor
{
public:
	field_sensor(b2World& world, entity::type, entity::id id, const b2Vec2& position);
	~field_sensor() noexcept;
private:
	b2World& world_;
	b2Body* body_;
	b2Fixture* fixture_;
};

field_sensor::field_sensor(b2World& world, entity::type type, entity::id id, const b2Vec2& position)
	:
		world_(world),
		body_(nullptr),
		fixture_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_staticBody;
	body_def.position.Set(position.x, position.y);
	body_def.angle = 0;
	body_ = world_.CreateBody(&body_def);

	b2CircleShape sensor;
	sensor.m_p.Set(0.0, 0.0);
	sensor.m_radius = 0.5;
	b2FixtureDef fixture_def;
	fixture_def.shape = &sensor;
	fixture_def.isSensor = true;
	entity::config_fixture(fixture_def, type, id);
	fixture_def.filter.maskBits = entity::type2category(entity::player_sensor);
	fixture_ = body_->CreateFixture(&fixture_def);
}

field_sensor::~field_sensor() noexcept
{
	try
	{
		world_.DestroyBody(body_);
	}
	catch (...)
	{
		// do nothing
	}
}

struct center_circle
{
	center_circle() = delete;
	center_circle(b2World& world, const b2Vec2& center_position, std::uint8_t radius = 20U);
	field_sensor spot;
	field_sensor top;
	field_sensor left;
	field_sensor right;
	field_sensor bottom;
};

center_circle::center_circle(b2World& world, const b2Vec2& center_position, std::uint8_t radius)
	:
		spot(world, entity::center_circle, entity::spot, center_position),
		top(world, entity::center_circle, entity::top, b2Vec2(center_position.x, center_position.y - radius)),
		left(world, entity::center_circle, entity::left, b2Vec2(center_position.x - radius, center_position.y)),
		right(world, entity::center_circle, entity::right, b2Vec2(center_position.x + radius, center_position.y)),
		bottom(world, entity::center_circle, entity::bottom, b2Vec2(center_position.x, center_position.y + radius))
{}

struct boundary
{
	boundary() = delete;
	explicit boundary(b2World& world);
	field_sensor top_left;
	field_sensor top_right;
	field_sensor bottom_left;
	field_sensor bottom_right;
	field_sensor half_line_top;
	field_sensor half_line_bottom;
	b2Fixture* top_sensor;
	b2Fixture* left_top_sensor;
	b2Fixture* left_bottom_sensor;
	b2Fixture* right_top_sensor;
	b2Fixture* right_bottom_sensor;
	b2Fixture* bottom_sensor;
private:
	b2Body* body_;
};

boundary::boundary(b2World& world)
	:
		top_left(world, entity::boundary, entity::top_left, b2Vec2(-120, 135)),
		top_right(world, entity::boundary, entity::top_right, b2Vec2(120, 135)),
		bottom_left(world, entity::boundary, entity::bottom_left, b2Vec2(-120, 15)),
		bottom_right(world, entity::boundary, entity::bottom_right, b2Vec2(120, 15)),
		half_line_top(world, entity::boundary, entity::top, b2Vec2(0, 135)),
		half_line_bottom(world, entity::boundary, entity::bottom, b2Vec2(0, 15)),
		top_sensor(nullptr),
		left_top_sensor(nullptr),
		left_bottom_sensor(nullptr),
		right_top_sensor(nullptr),
		right_bottom_sensor(nullptr),
		bottom_sensor(nullptr),
		body_(nullptr)
{
	b2BodyDef body_def;
	body_def.type = b2_staticBody;
	body_def.position.Set(0, 0);
	body_def.angle = 0;
	body_ = world.CreateBody(&body_def);

	std::array<b2Vec2, 4> top_vertices;
	top_vertices[0].Set(-122, 139);
	top_vertices[1].Set(122, 139);
	top_vertices[2].Set(122, 137);
	top_vertices[3].Set(-122, 137);
	b2PolygonShape top_box;
	top_box.Set(top_vertices.data(), top_vertices.max_size());
	b2FixtureDef top_def;
	top_def.shape = &top_box;
	top_def.density = 0;
	top_def.isSensor = true;
	entity::config_fixture(top_def, entity::boundary_sensor, entity::top);
	top_def.filter.maskBits = entity::type2category(entity::ball);
	top_sensor = body_->CreateFixture(&top_def);

	std::array<b2Vec2, 4> left_top_vertices;
	left_top_vertices[0].Set(-124, 139);
	left_top_vertices[1].Set(-122, 139);
	left_top_vertices[2].Set(-122, 87);
	left_top_vertices[3].Set(-124, 87);
	b2PolygonShape left_top_box;
	left_top_box.Set(left_top_vertices.data(), left_top_vertices.max_size());
	b2FixtureDef left_top_def;
	left_top_def.shape = &left_top_box;
	left_top_def.density = 0;
	left_top_def.isSensor = true;
	entity::config_fixture(left_top_def, entity::boundary_sensor, entity::top_left);
	left_top_def.filter.maskBits = entity::type2category(entity::ball);
	left_top_sensor = body_->CreateFixture(&left_top_def);

	std::array<b2Vec2, 4> left_bottom_vertices;
	left_bottom_vertices[0].Set(-124, 63);
	left_bottom_vertices[1].Set(-122, 63);
	left_bottom_vertices[2].Set(-122, 11);
	left_bottom_vertices[3].Set(-124, 11);
	b2PolygonShape left_bottom_box;
	left_bottom_box.Set(left_bottom_vertices.data(), left_bottom_vertices.max_size());
	b2FixtureDef left_bottom_def;
	left_bottom_def.shape = &left_bottom_box;
	left_bottom_def.density = 0;
	left_bottom_def.isSensor = true;
	entity::config_fixture(left_bottom_def, entity::boundary_sensor, entity::bottom_left);
	left_bottom_def.filter.maskBits = entity::type2category(entity::ball);
	left_bottom_sensor = body_->CreateFixture(&left_bottom_def);

	std::array<b2Vec2, 4> right_top_vertices;
	right_top_vertices[0].Set(122, 139);
	right_top_vertices[1].Set(124, 139);
	right_top_vertices[2].Set(124, 87);
	right_top_vertices[3].Set(122, 87);
	b2PolygonShape right_top_box;
	right_top_box.Set(right_top_vertices.data(), right_top_vertices.max_size());
	b2FixtureDef right_top_def;
	right_top_def.shape = &right_top_box;
	right_top_def.density = 0;
	right_top_def.isSensor = true;
	entity::config_fixture(right_top_def, entity::boundary_sensor, entity::top_right);
	right_top_def.filter.maskBits = entity::type2category(entity::ball);
	right_top_sensor = body_->CreateFixture(&right_top_def);

	std::array<b2Vec2, 4> right_bottom_vertices;
	right_bottom_vertices[0].Set(122, 63);
	right_bottom_vertices[1].Set(124, 63);
	right_bottom_vertices[2].Set(124, 11);
	right_bottom_vertices[3].Set(122, 11);
	b2PolygonShape right_bottom_box;
	right_bottom_box.Set(right_bottom_vertices.data(), right_bottom_vertices.max_size());
	b2FixtureDef right_bottom_def;
	right_bottom_def.shape = &right_bottom_box;
	right_bottom_def.density = 0;
	right_bottom_def.isSensor = true;
	entity::config_fixture(right_bottom_def, entity::boundary_sensor, entity::bottom_right);
	right_bottom_def.filter.maskBits = entity::type2category(entity::ball);
	right_bottom_sensor = body_->CreateFixture(&right_bottom_def);

	std::array<b2Vec2, 4> bottom_vertices;
	bottom_vertices[0].Set(-122, 13);
	bottom_vertices[1].Set(122, 13);
	bottom_vertices[2].Set(122, 11);
	bottom_vertices[3].Set(-122, 11);
	b2PolygonShape bottom_box;
	bottom_box.Set(bottom_vertices.data(), bottom_vertices.max_size());
	b2FixtureDef bottom_def;
	bottom_def.shape = &bottom_box;
	bottom_def.density = 0;
	bottom_def.isSensor = true;
	entity::config_fixture(bottom_def, entity::boundary_sensor, entity::bottom);
	bottom_def.filter.maskBits = entity::type2category(entity::ball);
	bottom_sensor = body_->CreateFixture(&bottom_def);
}

struct penalty_box
{
	penalty_box() = delete;
	penalty_box(b2World& world, entity::type type);
	float orientation;
	field_sensor spot;
	field_sensor top_left;
	field_sensor top_right;
	field_sensor bottom_left;
	field_sensor bottom_right;
};

penalty_box::penalty_box(b2World& world, entity::type type)
	:
		orientation((type == entity::left_penalty_box) ? -1 : 1),
		spot(world, type, entity::spot, b2Vec2(95 * orientation, 75)),
		top_left(world, type, entity::top_left, b2Vec2(120 * orientation, 115)),
		top_right(world, type, entity::top_right, b2Vec2(80 * orientation, 115)),
		bottom_left(world, type, entity::bottom_left, b2Vec2(120 * orientation, 35)),
		bottom_right(world, type, entity::bottom_right, b2Vec2(80 * orientation, 35))
{}

struct field
{
	field() = delete;
	explicit field(b2World& world);
	goal left_goal;
	goal right_goal;
	center_circle center;
	boundary boundary;
	penalty_box left_penalty;
	penalty_box right_penalty;
};

field::field(b2World& world)
	:
		left_goal(world, entity::left_goal, b2Vec2(-120, 75)),
		right_goal(world, entity::right_goal, b2Vec2(120, 75)),
		center(world, b2Vec2(0, 75)),
		boundary(world),
		left_penalty(world, entity::left_penalty_box),
		right_penalty(world, entity::right_penalty_box)
{}

struct sensor_event
{
	sensor_event();
	sensor_event(entity::type t, entity::id i, float d, float a);
	entity::type type;
	entity::id id;
	float distance;
	float angle;
};

sensor_event::sensor_event()
	:
		type(entity::player_sensor),
		id(entity::alpha_1),
		distance(0.0),
		angle(0.0)
{}

sensor_event::sensor_event(entity::type t, entity::id i, float d, float a)
	:
		type(t),
		id(i),
		distance(d),
		angle(a)
{}

class football : public Test
{
public:
	football();
	virtual void Step(Settings* settings);
	virtual void Keyboard(int key);
	static Test* Create();
	inline void increment_score() { ++score_; }
	inline void set_event(const sensor_event& event) { event_ = event; }
private:
	class contact_listener : public b2ContactListener
	{
	public:
		explicit contact_listener(football& football);
		virtual void BeginContact(b2Contact* contact);
	private:
		football& football_;
	};
	ball ball_;
	player player_a01_;
	player player_a02_;
	player player_a03_;
	player player_a04_;
	player player_a05_;
	player player_a06_;
	player player_a07_;
	player player_a08_;
	player player_a09_;
	player player_a10_;
	player player_a11_;
	player player_b01_;
	player player_b02_;
	player player_b03_;
	player player_b04_;
	player player_b05_;
	player player_b06_;
	player player_b07_;
	player player_b08_;
	player player_b09_;
	player player_b10_;
	player player_b11_;
	field field_;
	contact_listener listener_;
	std::uint8_t score_;
	sensor_event event_;
};

football::football()
	:
		Test(),
		ball_(*m_world, b2Vec2(0, 75)),
		player_a01_(*m_world, entity::alpha_1, b2Vec2(-10, 15), 90U),
		player_a02_(*m_world, entity::alpha_2, b2Vec2(-20, 15), 90U),
		player_a03_(*m_world, entity::alpha_3, b2Vec2(-30, 15), 90U),
		player_a04_(*m_world, entity::alpha_4, b2Vec2(-40, 15), 90U),
		player_a05_(*m_world, entity::alpha_5, b2Vec2(-50, 15), 90U),
		player_a06_(*m_world, entity::alpha_6, b2Vec2(-60, 15), 90U),
		player_a07_(*m_world, entity::alpha_7, b2Vec2(-70, 15), 90U),
		player_a08_(*m_world, entity::alpha_8, b2Vec2(-80, 15), 90U),
		player_a09_(*m_world, entity::alpha_9, b2Vec2(-90, 15), 90U),
		player_a10_(*m_world, entity::alpha_10, b2Vec2(-100, 15), 90U),
		player_a11_(*m_world, entity::alpha_11, b2Vec2(-110, 15), 90U),
		player_b01_(*m_world, entity::beta_1, b2Vec2(10, 15), 90U),
		player_b02_(*m_world, entity::beta_2, b2Vec2(20, 15), 90U),
		player_b03_(*m_world, entity::beta_3, b2Vec2(30, 15), 90U),
		player_b04_(*m_world, entity::beta_4, b2Vec2(40, 15), 90U),
		player_b05_(*m_world, entity::beta_5, b2Vec2(50, 15), 90U),
		player_b06_(*m_world, entity::beta_6, b2Vec2(60, 15), 90U),
		player_b07_(*m_world, entity::beta_7, b2Vec2(70, 15), 90U),
		player_b08_(*m_world, entity::beta_8, b2Vec2(80, 15), 90U),
		player_b09_(*m_world, entity::beta_9, b2Vec2(90, 15), 90U),
		player_b10_(*m_world, entity::beta_10, b2Vec2(100, 15), 90U),
		player_b11_(*m_world, entity::beta_11, b2Vec2(110, 15), 90U),
		field_(*m_world),
		listener_(*this),
		score_(0),
		event_()
{
	m_world->SetGravity(b2Vec2(0, 0));
	m_world->SetContactListener(&listener_);
}

void football::Step(Settings* settings)
{
	Test::Step(settings);
	g_debugDraw.DrawString(5, m_textLine, "Score: %d", score_);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player head angle: %3.2f", player_a01_.get_head_angle() * rad2deg);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player head velocity: %3.2f", player_a01_.get_head_velocity());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player torso angle: %3.2f", player_a01_.get_torso_angle() * rad2deg);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player torso angular velocity: %3.2f", player_a01_.get_torso_angular_velocity());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player foot translation: %3.2f", player_a01_.get_foot_translation());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Player foot speed: %3.2f", player_a01_.get_foot_speed());
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Sensed entity id: %d", event_.id);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Sensed entity type: %d", event_.type);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Sensed entity angle: %3.2f", event_.angle * rad2deg);
	m_textLine += DRAW_STRING_NEW_LINE;
	g_debugDraw.DrawString(5, m_textLine, "Sensed entity distance: %3.2f", event_.distance);
	m_textLine += DRAW_STRING_NEW_LINE;
}

void football::Keyboard(int key)
{
	switch (key)
	{
		case GLFW_KEY_Q:
			player_a01_.turn_head(100);
			break;
		case GLFW_KEY_E:
			player_a01_.turn_head(-100);
			break;
		case GLFW_KEY_W:
			player_a01_.run(8000);
			break;
		case GLFW_KEY_S:
			player_a01_.run(-8000);
			break;
		case GLFW_KEY_A:
			player_a01_.turn_torso(200);
			break;
		case GLFW_KEY_D:
			player_a01_.turn_torso(-200);
			break;
		case GLFW_KEY_3:
			player_a01_.kick(8000);
			break;
		case GLFW_KEY_1:
			player_a01_.kick(-8000);
			break;
	}
}

Test* football::Create()
{
	return new football();
}

football::contact_listener::contact_listener(football& football)
	:
		b2ContactListener(),
		football_(football)
{}

void football::contact_listener::BeginContact(b2Contact* contact)
{
	if (!contact || !contact->IsTouching())
	{
		return;
	}
	entity::type a_type = entity::get_type(*contact->GetFixtureA());
	entity::type b_type = entity::get_type(*contact->GetFixtureB());
	if ((a_type == entity::ball && b_type == entity::goal_sensor) ||
		(a_type == entity::goal_sensor && b_type == entity::ball))
	{
		football_.increment_score();
	}
	else if (a_type == entity::player_sensor)
	{
		b2WorldManifold manifold;
		contact->GetWorldManifold(&manifold);
		const b2Vec2& position_a = contact->GetFixtureA()->GetBody()->GetPosition();
		const b2Vec2& position_b = contact->GetFixtureB()->GetBody()->GetPosition();
		float distance = sqrtf(((position_b.x - position_a.x) * (position_b.x - position_a.x)) + ((position_b.y - position_a.y) * (position_b.y - position_a.y)));
		float entity_angle = atan2f(position_b.y - position_a.y, position_b.x - position_a.x);
		float head_angle = contact->GetFixtureA()->GetBody()->GetAngle();
		sensor_event event
		{
			b_type,
			entity::get_id(*contact->GetFixtureB()),
			distance,
			entity_angle - head_angle
		};
		football_.set_event(event);
	}
	else if (b_type == entity::player_sensor)
	{
		b2WorldManifold manifold;
		contact->GetWorldManifold(&manifold);
		const b2Vec2& position_a = contact->GetFixtureA()->GetBody()->GetPosition();
		const b2Vec2& position_b = contact->GetFixtureB()->GetBody()->GetPosition();
		float distance = sqrtf(((position_a.x - position_b.x) * (position_a.x - position_b.x)) + ((position_a.y - position_b.y) * (position_a.y - position_b.y)));
		float entity_angle = atan2f(position_a.y - position_b.y, position_a.x - position_b.x);
		float head_angle = contact->GetFixtureA()->GetBody()->GetAngle();
		sensor_event event
		{
			a_type,
			entity::get_id(*contact->GetFixtureA()),
			distance,
			entity_angle - head_angle
		};
		football_.set_event(event);
	}
}

#endif