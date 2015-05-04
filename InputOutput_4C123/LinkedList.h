#include "OS.h"
int LLAdd(tcbType** first, tcbType* insert, tcbType** last);

int LLRemove(tcbType** first, tcbType* insert, tcbType** last);


// insert into the sema4 linked list, insert at back of list
// need to modify this for a priority sema4Add
void Sem4LLAdd(tcbType** ptFrontPt,tcbType* insert,tcbType** ptEndPt);

// remove from the sema4 linked list, remove at front of list
tcbType* Sem4LLARemove(Sema4Type *semaPt);




