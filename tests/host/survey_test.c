#include <assert.h>
#include <stdio.h>
#include "survey.h"
int main(void) {
 for(unsigned zone=1;zone<=2;++zone) {
  Survey_Init(zone);
  unsigned edges=0;
  for(unsigned i=0;i<Survey_Count();++i) {
   MapPoint a,b; unsigned edge;
   assert(Survey_Leg(i,&a,&b,&edge));
   assert((a.x==b.x)!=(a.y==b.y));
   edges |= 1U<<edge;
  }
  assert(edges==0xfff && Survey_Count()==16); /* Chinese postman: 12 + 4 repeated */
  MapPoint a,b,c,d; unsigned e;
  assert(Survey_Leg(0,&a,&b,&e) && Survey_Leg(Survey_Count()-1,&c,&d,&e));
  assert(a.x==d.x && a.y==d.y); /* closed tour back to the gateway */
  for(unsigned i=1;i<Survey_Count();++i) {
   assert(Survey_Leg(i-1,&a,&b,&e) && Survey_Leg(i,&c,&d,&e));
   assert(b.x==c.x && b.y==c.y); /* consecutive legs connect */
  }
 }
 puts("survey coverage OK");
}
