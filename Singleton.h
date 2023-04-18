#pragma once
#ifndef __Singleton__
#define __Singleton__

#include <stdio.h>

template <typename T>
class Singleton {
public:
    static inline T& getInstance()
    {
        static T instance;
        return instance;
    }

protected:
    Singleton() {}
    virtual ~Singleton() {}

private:
    Singleton(const Singleton& rhs);
    Singleton& operator=(const Singleton& rhs);
};

#endif /* defined(__Singleton__) */