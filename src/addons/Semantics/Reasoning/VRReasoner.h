#ifndef VRREASONER_H_INCLUDED
#define VRREASONER_H_INCLUDED

#include <OpenSG/OSGConfig.h>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <memory>

using namespace std;

#include "VROntology.h"

struct VPath {
    string first;
    string root;
    vector<string> nodes;

    VPath(string p);
    string toString();
};

struct Variable {
    map<int, VREntityPtr> instances;
    string value;
    string concept;
    bool isAssumption = false;
    bool isAnonymous = true;
    bool valid = false;

    Variable();

    string toString();
    bool has(std::shared_ptr<Variable> other, VROntologyPtr onto);

    Variable(VROntologyPtr onto, string concept, string var);
    Variable(VROntologyPtr onto, string val);

    static std::shared_ptr<Variable> create(VROntologyPtr onto, string concept, string var);
    static std::shared_ptr<Variable> create(VROntologyPtr onto, string val);

    bool operator==(Variable v);
    void discard(VREntityPtr e);
};

typedef std::shared_ptr<Variable> VariablePtr;
typedef std::weak_ptr<Variable> VariableWeakPtr;

struct Result {
    vector<VREntityPtr> instances;
};

struct Term {
    VPath path;
    VariablePtr var;
    string str;

    Term(string s);
    bool valid();
    bool operator==(Term& other);
};

struct Statement;
typedef std::shared_ptr<Statement> StatementPtr;
typedef std::weak_ptr<Statement> StatementWeakPtr;

struct Statement {
    string verb;
    string verb_suffix;
    vector<Term> terms;
    int state = 0;
    int place = -1;

    Statement();
    Statement(string s, int i = -1);
    static StatementPtr New(string s, int i = -1);

    string toString();
    void updateLocalVariables(map<string, VariablePtr>& globals, VROntologyPtr onto);
    bool isSimpleVerb();
    bool match(StatementPtr s);
};

struct Query {
    StatementPtr request;
    vector<StatementPtr> statements;

    Query();
    Query(string q);
    string toString();

    void checkState();
};

struct Context {
    map<string, VariablePtr> vars;
    map<string, Result> results;
    map<string, Query> rules;
    list<Query> queries;
    VROntologyPtr onto = 0;

    int itr=0;
    int itr_max = 5;

    Context(VROntologyPtr onto);
    Context();
};

class VRReasoner {
    public:
        string pre = "  ?!?  ";
        string redBeg  = "\033[0;38;2;255;150;150m";
        string greenBeg = "\033[0;38;2;150;255;150m";
        string blueBeg = "\033[0;38;2;150;150;255m";
        string yellowBeg = "\033[0;38;2;255;255;150m";
        string colEnd = "\033[0m";

        static vector<string> split(string s, string d);
        static vector<string> split(string s, char d);
        static bool startswith(string s, string subs);

    private:
        VRReasoner();

        bool evaluate(StatementPtr s, Context& c);
        bool apply(StatementPtr s, Context& c);
        bool is(StatementPtr s, Context& c);
        bool has(StatementPtr s, Context& c);
        bool findRule(StatementPtr s, Context& c);

    public:
        static VRReasonerPtr create();
        vector<Result> process(string query, VROntologyPtr onto);
};


#endif // VRREASONER_H_INCLUDED
