#ifndef FOOTBALL_H
#define FOOTBALL_H

class Football : public Test
{
public:
	static Test* Create();
private:
};

Test* Football::Create()
{
	return new Football();
}

#endif