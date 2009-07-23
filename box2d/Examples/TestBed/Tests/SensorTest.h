/*
* Copyright (c) 2008-2009 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

// This is used to test sensor shapes.
class SensorTest : public Test
{
public:

	enum
	{
		e_count = 7
	};

	SensorTest()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);

			b2Body* ground = m_world->CreateBody(&bd);

			{
				b2PolygonDef sd;
				sd.SetAsBox(50.0f, 10.0f);
				ground->CreateFixture(&sd);
			}

#if 0
			{
				b2PolygonDef sd;
				sd.SetAsBox(10.0f, 2.0f, b2Vec2(0.0f, 20.0f), 0.0f);
				sd.isSensor = true;
				m_sensor = ground->CreateFixture(&sd);
			}
#else
			{
				b2CircleDef cd;
				cd.isSensor = true;
				cd.radius = 5.0f;
				cd.localPosition.Set(0.0f, 20.0f);
				m_sensor = ground->CreateFixture(&cd);
			}
#endif
		}

		{
			b2CircleDef sd;
			sd.radius = 1.0f;
			sd.density = 1.0f;

			for (int32 i = 0; i < e_count; ++i)
			{
				b2BodyDef bd;
				bd.position.Set(-10.0f + 3.0f * i, 20.0f);
				bd.userData = m_touching + i;

				m_touching[i] = false;
				m_bodies[i] = m_world->CreateBody(&bd);

				m_bodies[i]->CreateFixture(&sd);
				m_bodies[i]->SetMassFromShapes();
			}
		}
	}

	// Implement contact listener.
	void BeginContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			bool* touching = (bool*)userData;
			if(touching)
				*touching = true;
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			bool* touching = (bool*)userData;
			if(touching)
				*touching = true;
		}

		b2Fixture* fixture = NULL;
		if(contact->GetFixtureA()->IsSensor())
		{
			fixture = contact->GetFixtureB();
		}
		else if(contact->GetFixtureB()->IsSensor())
		{
			fixture = contact->GetFixtureA();
		}
		if(fixture)
		{
			b2CircleDef s;
			s.radius = 0.7f;
			s.density = 5;
			b2BodyDef d;
			b2Body* b;
			b2MassData md;
			switch(rand()%8)
			{
			case 0:
			case 1:
				s.localPosition = b2Vec2(RandomFloat(), RandomFloat());
				fixture->GetBody()->CreateFixture(&s);
				break;
			case 2:
			case 3:
				d.position = b2Vec2(RandomFloat() + 1, RandomFloat());
				b = m_world->CreateBody(&d);
				b->CreateFixture(&s);
				b->SetMassFromShapes();
				break;
			case 4:
				md.mass = 0;
				md.I = 0;
				md.center = b2Vec2(0.0f,0.0f);
				fixture->GetBody()->SetMassData(&md);
				fixture->GetBody()->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
				fixture->GetBody()->SetAngularVelocity(0.0f);
				break;
			case 5:
				m_world->DestroyBody(fixture->GetBody());
				break;
			case 6:
			case 7:
				b2Vec2 pos = b2Vec2(RandomFloat()*10,RandomFloat()*10)+fixture->GetBody()->GetPosition();
				fixture->GetBody()->SetXForm(pos,0.0f);
				break;
			}
		}
	}

	// Implement contact listener.
	void EndContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			bool* touching = (bool*)userData;
			if(touching)
				*touching = false;
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			bool* touching = (bool*)userData;
			if(touching)
				*touching = false;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int32 i = 0; i < e_count; ++i)
		{
			if (m_touching[i] == false)
			{
				continue;
			}

			b2Body* body = m_bodies[i];
			b2Body* ground = m_sensor->GetBody();

			b2CircleShape* circle = (b2CircleShape*)m_sensor->GetShape();
			b2Vec2 center = ground->GetWorldPoint(circle->m_p);

			b2Vec2 position = body->GetPosition();

			b2Vec2 d = center - position;
			if (d.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
			{
				continue;
			}

			d.Normalize();
			b2Vec2 F = 100.0f * d;
			body->ApplyForce(F, position);
		}
	}

	static Test* Create()
	{
		return new SensorTest;
	}

	b2Fixture* m_sensor;
	b2Body* m_bodies[e_count];
	bool m_touching[e_count];
};

#endif
