#pragma once

#include "physics/contact.h"

#include <vector>

class ContactSolver {
public:
    void solve(std::vector<Contact>& contacts, int iterations);

private:
    void matchWithPrevious(std::vector<Contact>& contacts);
    void warmStart(std::vector<Contact>& contacts);
    void solveContact(Contact& contact);
    void correctPosition(Contact& contact);

    std::vector<Contact> previousContacts_;

public:
    void clearHistory() { previousContacts_.clear(); }
};
