/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */

#ifndef __CLI_LIST_H__
#define __CLI_LIST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "cli_porting.h"

struct ListHead {
    struct ListHead *next, *prev;
};

struct HlistNode {
    struct HlistNode *next, **pprev;
};

struct HlistHead {
    struct HlistNode *first;
};

#define WRITE_ONCE(var, val) \
        (*((volatile typeof(val) *)(&(var))) = (val))

#define READ_ONCE(var) \
        (*((volatile typeof(var) *)(&(var))))

#define LIST_POISON1  NULL
#define LIST_POISON2  NULL

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
    struct ListHead name = LIST_HEAD_INIT(name)

static inline void initListHead(struct ListHead *list)
{
    WRITE_ONCE(list->next, list);
    list->prev = list;
}

static inline Bool listAddValid(struct ListHead *curr,
            struct ListHead *prev,
            struct ListHead *next)
{
    return true;
}

static inline Bool listDelEntryValid(struct ListHead *entry)
{
    return true;
}

/*
 * Insert a curr entry between two known consecutive entries.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static inline void __listAdd(struct ListHead *curr,
                  struct ListHead *prev,
                  struct ListHead *next)
{
    if (!listAddValid(curr, prev, next))
        return;

    next->prev = curr;
    curr->next = next;
    curr->prev = prev;
    WRITE_ONCE(prev->next, curr);
}

/**
 * listAdd - add a curr entry
 * @curr: curr entry to be added
 * @head: list head to add it after
 *
 * Insert a curr entry after the specified head.
 * This is good for implementing stacks.
 */
static inline void listAdd(struct ListHead *curr, struct ListHead *head)
{
    __listAdd(curr, head, head->next);
}

/**
 * listAddTail - add a curr entry
 * @curr: curr entry to be added
 * @head: list head to add it before
 *
 * Insert a curr entry before the specified head.
 * This is useful for implementing queues.
 */
static inline void listAddTail(struct ListHead *curr, struct ListHead *head)
{
    __listAdd(curr, head->prev, head);
}

/**
 * listInsertHead - add a new entry
 * @new: new entry to be added
 * @head: list head to add it before
 *
 * Insert a new entry after the specified head.
 * This is useful for implementing queues.
 */
static inline void listInsertHead(struct ListHead *curr, struct ListHead *head)
{
    __listAdd(curr, head, head->next);
}

/*
 * Delete a list entry by making the prev/next entries
 * point to each other.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static inline void __listDel(struct ListHead *prev, struct ListHead * next)
{
    next->prev = prev;
    WRITE_ONCE(prev->next, next);
}

/**
 * listDel - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: listEmpty() on entry does not return true after this, the entry is
 * in an undefined state.
 */
static inline void listDelEntry(struct ListHead *entry)
{
    if (!listDelEntryValid(entry))
        return;

    __listDel(entry->prev, entry->next);
}

static inline void listDel(struct ListHead *entry)
{
    listDelEntry(entry);
    entry->next = LIST_POISON1;
    entry->prev = LIST_POISON2;
}

/**
 * listReplace - replace old entry by curr one
 * @old : the element to be replaced
 * @curr : the curr element to insert
 *
 * If @old was empty, it will be overwritten.
 */
static inline void listReplace(struct ListHead *old,
                struct ListHead *curr)
{
    curr->next = old->next;
    curr->next->prev = curr;
    curr->prev = old->prev;
    curr->prev->next = curr;
}

static inline void listReplaceInit(struct ListHead *old,
                    struct ListHead *curr)
{
    listReplace(old, curr);
    initListHead(old);
}

/**
 * listDelInit - deletes entry from list and reinitialize it.
 * @entry: the element to delete from the list.
 */
static inline void listDelInit(struct ListHead *entry)
{
    listDelEntry(entry);
    initListHead(entry);
}

/**
 * listMove - delete from one list and add as another's head
 * @list: the entry to move
 * @head: the head that will precede our entry
 */
static inline void listMove(struct ListHead *list, struct ListHead *head)
{
    listDelEntry(list);
    listAdd(list, head);
}

/**
 * listMoveTail - delete from one list and add as another's tail
 * @list: the entry to move
 * @head: the head that will follow our entry
 */
static inline void listMoveTail(struct ListHead *list,
                  struct ListHead *head)
{
    listDelEntry(list);
    listAddTail(list, head);
}

/**
 * listBulkMoveTail - move a subsection of a list to its tail
 * @head: the head that will follow our entry
 * @first: first entry to move
 * @last: last entry to move,can be the same as first
 *
 * Move all entries between @first and includeing @last before @head.
 * All three entries must belong to the same linked list.
 */
static inline void listBulkMoveTail(struct ListHead *head,
                    struct ListHead *first,
                    struct ListHead *last)
{
    first->prev->next = last->next;
    last->next->prev = first->prev;

    head->prev->next = first;
    first->prev = head->prev;

    last->next = head;
    head->prev = last;
}

/**
 * listIsLast - tests whether @list is the last entry in list @head
 * @list: the entry to test
 * @head: the head of the list
 */
static inline Bool listIsLast(const struct ListHead *list,
                const struct ListHead *head)
{
    return (Bool)(list->next == head);
}

/**
 * listEmpty - tests whether a list is empty
 * @head: the list to test.
 */
static inline Bool listEmpty(const struct ListHead *head)
{
    return (Bool)(READ_ONCE(head->next) == head);
}

static inline struct ListHead *listGetNext( const struct ListHead *head )
{
    return head->next;
}
static inline Bool listIsAfterLastNode( const struct ListHead *pList, const struct ListHead *pNode )
{
    return (Bool)(pNode == pList);
}

/**
 * listEmptyCareful - tests whether a list is empty and not being modified
 * @head: the list to test
 *
 * Description:
 * tests whether a list is empty _and_ checks that no other CPU might be
 * in the process of modifying either member (next or prev)
 *
 * NOTE: using listEmptyCareful() without synchronization
 * can only be safe if the only activity that can happen
 * to the list entry is listDelInit(). Eg. it cannot be used
 * if another CPU could re-listAdd() it.
 */
static inline Bool listEmptyCareful(const struct ListHead *head)
{
    struct ListHead *next = head->next;
    return (Bool)((next == head) && (next == head->prev));
}

/**
 * listRotateLeft - rotate the list to the left
 * @head: the head of the list
 */
static inline void listRotateLeft(struct ListHead *head)
{
    struct ListHead *first;

    if (!listEmpty(head)) {
        first = head->next;
        listMoveTail(first, head);
    }
}

/**
 * listIsSingular - tests whether a list has just one entry.
 * @head: the list to test.
 */
static inline Bool listIsSingular(const struct ListHead *head)
{
    return (Bool)(!listEmpty(head) && (head->next == head->prev));
}

static inline void __listCutPosition(struct ListHead *list,
        struct ListHead *head, struct ListHead *entry)
{
    struct ListHead *new_first = entry->next;
    list->next = head->next;
    list->next->prev = list;
    list->prev = entry;
    entry->next = list;
    head->next = new_first;
    new_first->prev = head;
}

/**
 * listCutPosition - cut a list into two
 * @list: a curr list to add all removed entries
 * @head: a list with entries
 * @entry: an entry within head, could be the head itself
 *    and if so we won't cut the list
 *
 * This helper moves the initial part of @head, up to and
 * including @entry, from @head to @list. You should
 * pass on @entry an element you know is on @head. @list
 * should be an empty list or a list you do not care about
 * losing its data.
 *
 */
static inline void listCutPosition(struct ListHead *list,
        struct ListHead *head, struct ListHead *entry)
{
    if (listEmpty(head))
        return;
    if (listIsSingular(head) &&
        (head->next != entry && head != entry))
        return;
    if (entry == head)
        initListHead(list);
    else
        __listCutPosition(list, head, entry);
}

static inline void __listSplice(const struct ListHead *list,
                 struct ListHead *prev,
                 struct ListHead *next)
{
    struct ListHead *first = list->next;
    struct ListHead *last = list->prev;

    first->prev = prev;
    prev->next = first;

    last->next = next;
    next->prev = last;
}

/**
 * listSplice - join two lists, this is designed for stacks
 * @list: the curr list to add.
 * @head: the place to add it in the first list.
 */
static inline void listSplice(const struct ListHead *list,
                struct ListHead *head)
{
    if (!listEmpty(list))
        __listSplice(list, head, head->next);
}

/**
 * listSpliceTail - join two lists, each list being a queue
 * @list: the curr list to add.
 * @head: the place to add it in the first list.
 */
static inline void listSpliceTail(struct ListHead *list,
                struct ListHead *head)
{
    if (!listEmpty(list))
        __listSplice(list, head->prev, head);
}

/**
 * listSpliceInit - join two lists and reinitialise the emptied list.
 * @list: the curr list to add.
 * @head: the place to add it in the first list.
 *
 * The list at @list is reinitialised
 */
static inline void listSpliceInit(struct ListHead *list,
                    struct ListHead *head)
{
    if (!listEmpty(list)) {
        __listSplice(list, head, head->next);
        initListHead(list);
    }
}

/**
 * listSpliceTailInit - join two lists and reinitialise the emptied list
 * @list: the curr list to add.
 * @head: the place to add it in the first list.
 *
 * Each of the lists is a queue.
 * The list at @list is reinitialised
 */
static inline void listSpliceTailInit(struct ListHead *list,
                     struct ListHead *head)
{
    if (!listEmpty(list)) {
        __listSplice(list, head->prev, head);
        initListHead(list);
    }
}

#define container_of(ptr, type, member) ({ \
    const typeof( ((type *)0)->member ) *__mptr = (ptr); \
    (type *)( (char *)__mptr - offsetof(type,member) );})

/**
 * LIST_ENTRY - get the struct for this entry
 * @ptr:    the &struct ListHead pointer.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_ENTRY(ptr, type, member) \
    container_of(ptr, type, member)

/**
 * LIST_FIRST_ENTRY - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the ListHead within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define LIST_FIRST_ENTRY(ptr, type, member) \
    LIST_ENTRY((ptr)->next, type, member)

/**
 * LIST_LAST_ENTRY - get the last element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the ListHead within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define LIST_LAST_ENTRY(ptr, type, member) \
    LIST_ENTRY((ptr)->prev, type, member)

/**
 * LIST_FIRST_ENTRY_OR_NULL - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the ListHead within the struct.
 *
 * Note that if the list is empty, it returns NULL.
 */
#define LIST_FIRST_ENTRY_OR_NULL(ptr, type, member) ({ \
    struct ListHead *head__ = (ptr); \
    struct ListHead *pos__ = READ_ONCE(head__->next); \
    pos__ != head__ ? LIST_ENTRY(pos__, type, member) : NULL; \
})

/**
 * LIST_NEXT_ENTRY - get the next element in list
 * @pos:    the type * to cursor
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_NEXT_ENTRY(pos, member) \
    LIST_ENTRY((pos)->member.next, typeof(*(pos)), member)

/**
 * LIST_PREV_ENTRY - get the prev element in list
 * @pos:    the type * to cursor
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_PREV_ENTRY(pos, member) \
    LIST_ENTRY((pos)->member.prev, typeof(*(pos)), member)

/**
 * LIST_FOR_EACH    -    iterate over a list
 * @pos:    the &struct ListHead to use as a loop cursor.
 * @head:    the head for your list.
 */
#define LIST_FOR_EACH(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * LIST_FOR_EACH_PREV    -    iterate over a list backwards
 * @pos:    the &struct ListHead to use as a loop cursor.
 * @head:    the head for your list.
 */
#define LIST_FOR_EACH_PREV(pos, head) \
    for (pos = (head)->prev; pos != (head); pos = pos->prev)

#define LIST_FOR_EACH_CONTINUE(pos, head) \
    for (pos = (pos)->next; pos != (head); pos = pos->next)

/**
 * LIST_FOR_EACH_SAFE - iterate over a list safe against removal of list entry
 * @pos:    the &struct ListHead to use as a loop cursor.
 * @n:        another &struct ListHead to use as temporary storage
 * @head:    the head for your list.
 */
#define LIST_FOR_EACH_SAFE(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

/**
 * LIST_FOR_EACH_PREV_SAFE - iterate over a list backwards safe against removal of list entry
 * @pos:    the &struct ListHead to use as a loop cursor.
 * @n:        another &struct ListHead to use as temporary storage
 * @head:    the head for your list.
 */
#define LIST_FOR_EACH_PREV_SAFE(pos, n, head) \
    for (pos = (head)->prev, n = pos->prev; \
         pos != (head); \
         pos = n, n = pos->prev)

/**
 * LIST_FOR_EACH_ENTRY    -    iterate over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_FOR_EACH_ENTRY(pos, head, member)                \
    for (pos = LIST_FIRST_ENTRY(head, typeof(*pos), member);    \
         &pos->member != (head);                    \
         pos = LIST_NEXT_ENTRY(pos, member))

/**
 * LIST_FOR_EACH_ENTRY_REVERSE - iterate backwards over list of given type.
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_FOR_EACH_ENTRY_REVERSE(pos, head, member)            \
    for (pos = LIST_LAST_ENTRY(head, typeof(*pos), member);        \
         &pos->member != (head);                     \
         pos = LIST_PREV_ENTRY(pos, member))

/**
 * LIST_PREPARE_ENTRY - prepare a pos entry for use in LIST_FOR_EACH_ENTRY_CONTINUE()
 * @pos:    the type * to use as a start point
 * @head:    the head of the list
 * @member:    the name of the ListHead within the struct.
 *
 * Prepares a pos entry for use as a start point in LIST_FOR_EACH_ENTRY_CONTINUE().
 */
#define LIST_PREPARE_ENTRY(pos, head, member) \
    ((pos) ? : LIST_ENTRY(head, typeof(*pos), member))

/**
 * LIST_FOR_EACH_ENTRY_CONTINUE - continue iteration over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Continue to iterate over list of given type, continuing after
 * the current position.
 */
#define LIST_FOR_EACH_ENTRY_CONTINUE(pos, head, member)         \
    for (pos = LIST_NEXT_ENTRY(pos, member);            \
         &pos->member != (head);                    \
         pos = LIST_NEXT_ENTRY(pos, member))

/**
 * LIST_FOR_EACH_ENTRY_CONTINUE_REVERSE - iterate backwards from the given point
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Start to iterate over list of given type backwards, continuing after
 * the current position.
 */
#define LIST_FOR_EACH_ENTRY_CONTINUE_REVERSE(pos, head, member)        \
    for (pos = LIST_PREV_ENTRY(pos, member);            \
         &pos->member != (head);                    \
         pos = LIST_PREV_ENTRY(pos, member))

/**
 * LIST_FOR_EACH_ENTRY_FROM - iterate over list of given type from the current point
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Iterate over list of given type, continuing from current position.
 */
#define LIST_FOR_EACH_ENTRY_FROM(pos, head, member)             \
    for (; &pos->member != (head);                    \
         pos = LIST_NEXT_ENTRY(pos, member))

/**
 * LIST_FOR_EACH_ENTRY_FROM_REVERSE - iterate backwards over list of given type
 *                                    from the current point
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Iterate backwards over list of given type, continuing from current position.
 */
#define LIST_FOR_EACH_ENTRY_FROM_REVERSE(pos, head, member)        \
    for (; &pos->member != (head);                    \
         pos = LIST_PREV_ENTRY(pos, member))

/**
 * LIST_FOR_EACH_ENTRY_SAFE - iterate over list of given type safe against removal of list entry
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 */
#define LIST_FOR_EACH_ENTRY_SAFE(pos, n, head, member)            \
    for (pos = LIST_FIRST_ENTRY(head, typeof(*pos), member),    \
        n = LIST_NEXT_ENTRY(pos, member);            \
         &pos->member != (head);                     \
         pos = n, n = LIST_NEXT_ENTRY(n, member))

/**
 * LIST_FOR_EACH_ENTRY_SAFE_CONTINUE - continue list iteration safe against removal
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Iterate over list of given type, continuing after current point,
 * safe against removal of list entry.
 */
#define LIST_FOR_EACH_ENTRY_SAFE_CONTINUE(pos, n, head, member)         \
    for (pos = LIST_NEXT_ENTRY(pos, member),                 \
        n = LIST_NEXT_ENTRY(pos, member);                \
         &pos->member != (head);                        \
         pos = n, n = LIST_NEXT_ENTRY(n, member))

/**
 * LIST_FOR_EACH_ENTRY_SAFE_FROM - iterate over list from current point safe against removal
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Iterate over list of given type from current point, safe against
 * removal of list entry.
 */
#define LIST_FOR_EACH_ENTRY_SAFE_FROM(pos, n, head, member)             \
    for (n = LIST_NEXT_ENTRY(pos, member);                    \
         &pos->member != (head);                        \
         pos = n, n = LIST_NEXT_ENTRY(n, member))

/**
 * LIST_FOR_EACH_ENTRY_SAFE_REVERSE - iterate backwards over list safe against removal
 * @pos:    the type * to use as a loop cursor.
 * @n:        another type * to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the ListHead within the struct.
 *
 * Iterate backwards over list of given type, safe against removal
 * of list entry.
 */
#define LIST_FOR_EACH_ENTRY_SAFE_REVERSE(pos, n, head, member)        \
    for (pos = LIST_LAST_ENTRY(head, typeof(*pos), member),        \
        n = LIST_PREV_ENTRY(pos, member);            \
         &pos->member != (head);                     \
         pos = n, n = LIST_PREV_ENTRY(n, member))
/**
 * LIST_SAFE_RESET_NEXT - reset a stale LIST_FOR_EACH_ENTRY_SAFE loop
 * @pos:    the loop cursor used in the LIST_FOR_EACH_ENTRY_SAFE loop
 * @n:        temporary storage used in LIST_FOR_EACH_ENTRY_SAFE
 * @member:    the name of the ListHead within the struct.
 *
 * LIST_SAFE_RESET_NEXT is not safe to use in general if the list may be
 * modified concurrently (eg. the lock is dropped in the loop body). An
 * exception to this is if the cursor element (pos) is pinned in the list,
 * and LIST_SAFE_RESET_NEXT is called after re-taking the lock and before
 * completing the current iteration of the loop body.
 */
#define LIST_SAFE_RESET_NEXT(pos, n, member)                \
    n = LIST_NEXT_ENTRY(pos, member)

/*
 * Double linked lists with a single pointer list head.
 * Mostly useful for hash tables where the two pointer list head is
 * too wasteful.
 * You lose the ability to access the tail in O(1).
 */

#define HLIST_HEAD_INIT { .first = NULL }
#define HLIST_HEAD(name) struct HlistHead name = {  .first = NULL }
#define INIT_HLIST_HEAD(ptr) ((ptr)->first = NULL)
static inline void initHlistNode(struct HlistNode *h)
{
    h->next = NULL;
    h->pprev = NULL;
}

static inline Bool hlistUnhashed(const struct HlistNode *h)
{
    return (Bool)(!h->pprev);
}

static inline Bool hlistEmpty(const struct HlistHead *h)
{
    return (Bool)(!READ_ONCE(h->first));
}

static inline void __hlistDel(struct HlistNode *n)
{
    struct HlistNode *next = n->next;
    struct HlistNode **pprev = n->pprev;

    WRITE_ONCE(*pprev, next);
    if (next)
        next->pprev = pprev;
}

static inline void hlistDel(struct HlistNode *n)
{
    __hlistDel(n);
    n->next = LIST_POISON1;
    n->pprev = LIST_POISON2;
}

static inline void hlistDelInit(struct HlistNode *n)
{
    if (!hlistUnhashed(n)) {
        __hlistDel(n);
        initHlistNode(n);
    }
}

static inline void hlistAddHead(struct HlistNode *n, struct HlistHead *h)
{
    struct HlistNode *first = h->first;
    n->next = first;
    if (first)
        first->pprev = &n->next;
    WRITE_ONCE(h->first, n);
    n->pprev = &h->first;
}

///< next must be != NULL
static inline void hlistAddBefore(struct HlistNode *n,
                    struct HlistNode *next)
{
    n->pprev = next->pprev;
    n->next = next;
    next->pprev = &n->next;
    WRITE_ONCE(*(n->pprev), n);
}

static inline void hlistAddBehind(struct HlistNode *n,
                    struct HlistNode *prev)
{
    n->next = prev->next;
    WRITE_ONCE(prev->next, n);
    n->pprev = &prev->next;

    if (n->next)
        n->next->pprev  = &n->next;
}

///< after that we'll appear to be on some hlist and hlistDel will work
static inline void hlistAddFake(struct HlistNode *n)
{
    n->pprev = &n->next;
}

static inline Bool hlistFake(struct HlistNode *h)
{
    return (Bool)(h->pprev == &h->next);
}

/*
 * Check whether the node is the only node of the head without
 * accessing head:
 */
static inline Bool
hlistIsSingularNode(struct HlistNode *n, struct HlistHead *h)
{
    return (Bool)(!n->next && n->pprev == &h->first);
}

/*
 * Move a list from one list head to another. Fixup the pprev
 * reference of the first entry if it exists.
 */
static inline void hlistMoveList(struct HlistHead *old,
                   struct HlistHead *curr)
{
    curr->first = old->first;
    if (curr->first)
        curr->first->pprev = &curr->first;
    old->first = NULL;
}

#define HLIST_ENTRY(ptr, type, member) container_of(ptr,type,member)

#define HLIST_FOR_EACH(pos, head) \
    for (pos = (head)->first; pos ; pos = pos->next)

#define HLIST_FOR_EACH_SAFE(pos, n, head) \
    for (pos = (head)->first; pos && ({ n = pos->next; 1; }); \
         pos = n)

#define HLIST_ENTRY_SAFE(ptr, type, member) \
    ({ typeof(ptr) ____ptr = (ptr); \
       ____ptr ? HLIST_ENTRY(____ptr, type, member) : NULL; \
    })

/**
 * HLIST_FOR_EACH_ENTRY    - iterate over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:    the head for your list.
 * @member:    the name of the HlistNode within the struct.
 */
#define HLIST_FOR_EACH_ENTRY(pos, head, member)                \
    for (pos = HLIST_ENTRY_SAFE((head)->first, typeof(*(pos)), member);\
         pos;                            \
         pos = HLIST_ENTRY_SAFE((pos)->member.next, typeof(*(pos)), member))

/**
 * HLIST_FOR_EACH_ENTRY_CONTINUE - iterate over a hlist continuing after current point
 * @pos:    the type * to use as a loop cursor.
 * @member:    the name of the HlistNode within the struct.
 */
#define HLIST_FOR_EACH_ENTRY_CONTINUE(pos, member)            \
    for (pos = HLIST_ENTRY_SAFE((pos)->member.next, typeof(*(pos)), member);\
         pos;                            \
         pos = HLIST_ENTRY_SAFE((pos)->member.next, typeof(*(pos)), member))

/**
 * HLIST_FOR_EACH_ENTRY_FROM - iterate over a hlist continuing from current point
 * @pos:    the type * to use as a loop cursor.
 * @member:    the name of the HlistNode within the struct.
 */
#define HLIST_FOR_EACH_ENTRY_FROM(pos, member)                \
    for (; pos;                            \
         pos = HLIST_ENTRY_SAFE((pos)->member.next, typeof(*(pos)), member))

/**
 * HLIST_FOR_EACH_ENTRY_SAFE - iterate over list of given type safe against removal of list entry
 * @pos:    the type * to use as a loop cursor.
 * @n:        another &struct HlistNode to use as temporary storage
 * @head:    the head for your list.
 * @member:    the name of the HlistNode within the struct.
 */
#define HLIST_FOR_EACH_ENTRY_SAFE(pos, n, head, member)         \
    for (pos = HLIST_ENTRY_SAFE((head)->first, typeof(*pos), member);\
         pos && ({ n = pos->member.next; 1; });            \
         pos = HLIST_ENTRY_SAFE(n, typeof(*pos), member))

#ifdef __cplusplus
}
#endif

#endif ///< __CLI_LIST_H__

