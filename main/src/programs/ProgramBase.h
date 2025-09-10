#pragma once

class ProgramBase {
public:
    virtual void init() = 0; // ce qui se fait avant le départ (tirette)
    virtual void run() = 0;  // ce qui se fait après le départ
    virtual ~ProgramBase() {}
};
