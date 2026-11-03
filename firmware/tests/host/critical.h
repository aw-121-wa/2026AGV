#ifndef HOST_CRITICAL_H
#define HOST_CRITICAL_H
typedef unsigned critical_state;
static inline critical_state Critical_Enter(void)
{
    return 0U;
}
static inline void Critical_Exit(critical_state s)
{
    (void)s;
}
#endif
