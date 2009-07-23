/*
* Copyright (c) 2007-2009 Erin Catto http://www.gphysics.com
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

#ifndef RAYCAST_TEST_H
#define RAYCAST_TEST_H

class RaycastTest : public Test
{
public:
	RaycastTest()
	{
		//m_world->SetGravity(b2Vec2(0,0));

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			ground = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			ground->CreateFixture(&sd);
		}

		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 1.0f);
			laserBody = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(5.0f, 1.0f);
			sd.density = 4.0;
			laserBody->CreateFixture(&sd);
			laserBody->SetMassFromShapes();

			b2Body* body;
			//Create a few shapes
			bd.position.Set(-5.0f, 10.0f);
			body = m_world->CreateBody(&bd);

			b2CircleDef cd;
			cd.radius = 3;
			body->CreateFixture(&cd);

			bd.position.Set(5.0f, 10.0f);
			body = m_world->CreateBody(&bd);

			body->CreateFixture(&cd);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 0:
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		float32 segmentLength = 30.0f;

		b2Segment segment;
		b2Vec2 laserStart(5.0f-0.1f,0.0f);
		b2Vec2 laserDir(segmentLength,0.0f);
		segment.p1 = laserBody->GetWorldPoint(laserStart);
		segment.p2 = laserBody->GetWorldVector(laserDir);
		segment.p2+=segment.p1;

		for(int32 rebounds=0;rebounds<10;rebounds++){

			float32 lambda=1;
			b2Vec2 normal;
			b2Fixture* fixture = m_world->RaycastOne(segment,&lambda,&normal,false,NULL);

			b2Color laserColor(255,0,0);

			if(fixture)
			{
				m_debugDraw.DrawSegment(segment.p1,(1-lambda)*segment.p1+lambda*segment.p2,laserColor);
			}
			else
			{
				m_debugDraw.DrawSegment(segment.p1,segment.p2,laserColor);
				break;
			}
			//Bounce
			segmentLength *=(1-lambda);
			if(segmentLength<=B2_FLT_EPSILON)
				break;
			laserStart = (1-lambda)*segment.p1+lambda*segment.p2;
			laserDir = segment.p2-segment.p1;
			laserDir.Normalize();
			laserDir = laserDir -2 * b2Dot(laserDir,normal) * normal;
			segment.p1 = laserStart-0.1f*laserDir;
			segment.p2 = laserStart+segmentLength*laserDir;
		}
	}

	static Test* Create()
	{
		return new RaycastTest;
	}

	b2Body* laserBody;

};

#endif
