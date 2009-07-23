#ifndef FIXED_JOINT_H
#define FIXED_JOINT_H

class FixedJoint : public Test
{
public:
	FixedJoint()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			b2Body* ground = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			ground->CreateFixture(&sd);
		}

		{
			b2BodyDef bd1;
			bd1.position.Set(1.0f, 7.0f);
			bd1.angle = 0.7f;
			b2Body* body1 = m_world->CreateBody(&bd1);

			b2PolygonDef sd1;
			sd1.vertexCount = 3;
			sd1.vertices[0].Set(-2.0f, -2.0f);
			sd1.vertices[1].Set(2.0f, -2.0f);
			sd1.vertices[2].Set(2.0f, 2.0f);
			sd1.density = 10.0f;
			sd1.friction = 0.2f;
			body1->CreateFixture(&sd1);
			body1->SetMassFromShapes();

			b2BodyDef bd2;
			bd2.position.Set(5.0f, 6.0f);
			bd2.angle = -0.2f;
			b2Body* body2 = m_world->CreateBody(&bd2);

			b2PolygonDef sd2;
			sd2.SetAsBox(1.0f, 2.0f);
			sd2.density = 30.0f;
			sd2.friction = 0.2f;
			body2->CreateFixture(&sd2);
			body2->SetMassFromShapes();

			b2FixedJointDef jd;
			jd.collideConnected = false;
			jd.Initialize(body1, body2);
			m_world->CreateJoint(&jd);
		}
	}

	static Test* Create()
	{
		return new FixedJoint;
	}
};

#endif
