/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_normal_output.c
 * @author    xiezhj
 * @date      2021.04.12
 * @brief     cli 格式化输出
 * @note
 */

#include "cli_normal_output.h"

static S8 sgStrBuf[STR_MAX_LEN + 1] = { 0 };
/**
 * node 相关操作接口
 */

///< Macros to compute minimum and maximum
#ifndef MAX
#define MAX(A, B) (((A) > (B)) ? (A) : (B))
#endif
#ifndef MIN
#define MIN(A, B) (((A) < (B)) ? (A) : (B))
#endif

/**
 * @brief  分配内存并创建一个新字符串
 * @param  str  输入字符串
 * @return char*
 */
static S8 *cliNewString(S8 *str)
{
    S32 size = 0;
    S8  *pTmp   = NULL;

    size = (S32)cliStrLen(str);
    pTmp = cliMalloc(size + 1, 0);
    if (NULL == pTmp) {
        CLI_LOG_ERROR("pTmp alloc fail.");
        goto lEnd;
    }
    (void)cliStrNCpy(pTmp, str, size + 1);
lEnd:
    return pTmp;
}

/**
 * @brief  创建一个新节点
 * @param  name   节点名
 * @return Node_t*
 */
static Node_s *cliNewNode(S8 *pName)
{
    Node_s *pNode = NULL;
    pNode = cliMalloc((S32)sizeof(*pNode), 0);
    if (NULL == pNode) {
        CLI_LOG_ERROR("pNode alloc failed.");
        goto lEnd;
    }

    if (pName != NULL) {
        pNode->pName = cliNewString(pName);
    }
lEnd:
    return pNode;
}

/**
 * @brief  释放字符串list
 * @param  stringList
 */
static void cliDeleteStringList(S8 ** stringList)
{
    U32 i = 0;

    for (i = 0; stringList[i] != NULL; i++) {
        cliFree(stringList[i]);
    }
}

/**
 * @brief  删除单一节点
 * @param  Node_t
 */
static void cliDeleteOneNode(Node_s *pNode)
{
    Node_s         *pTmp  = NULL;
    TableRowList_s *pList = NULL;
    TableRowList_s *pNext = NULL;
    NoteList_s     *pLst  = NULL;
    NoteList_s     *pNxt  = NULL;

    if (pNode->pParent != NULL) {
        if (pNode->pParent->pChild == pNode) {
            pNode->pParent->pChild = pNode->pNext;
        } else {
            pTmp = pNode->pParent->pChild;
            while (pTmp->pNext) {
                if (pTmp->pNext == pNode) {
                    pTmp->pNext = pNode->pNext;
                    break;
                }
                pTmp = pTmp->pNext;
            }
        }
    }

    if (pNode->pName != NULL) {
        cliFree(pNode->pName);
    }
    if (pNode->pValue != NULL) {
        cliFree(pNode->pValue);
    }
    if (pNode->pTable != NULL) {
        cliDeleteStringList(pNode->pTable->titles);
        cliFree(pNode->pTable->titles);
        for (pList = pNode->pTable->pRowList; pList != NULL;) {
            cliDeleteStringList(pList->row.pStrings);
            cliFree(pList->row.pStrings);
            pNext = pList->pNext;
            cliFree(pList);
            pList = pNext;
        }
        for (pLst = pNode->pTable->pNoteList; pLst != NULL;) {
            pNxt = pLst->pNext;
            if (pLst->note.pKey != NULL) {
                cliFree(pLst->note.pKey);
            }
            if (pLst->note.pValue != NULL) {
                cliFree(pLst->note.pValue);
            }
            cliFree(pLst);
            pLst = pNxt;
        }
        cliFree(pNode->pTable);
    }

    cliFree(pNode);
}

/**
 * @brief  释放节点空间
 * @param  node   节点
 */
void cliDeleteNode(Node_s *pNode)
{
    Node_s *pNodeIn = NULL;
    Node_s *pNodeTmp = NULL;

    ///< 入参检查
    if (pNode == NULL) {
        CLI_LOG_ERROR("input para is null.");
        goto lEnd;
    }
    pNodeIn = pNode;

    if (pNode->pChild != NULL) {
        pNode = pNode->pChild;
        while (pNode != pNodeIn) {
            pNodeTmp = pNode;

            if (pNode->pChild != NULL) { ///< 优先遍历子节点
                pNode = pNode->pChild;
            } else if (pNode->pNext != NULL) { ///< 遍历兄弟节点
                pNode = pNode->pNext;
                cliDeleteOneNode(pNodeTmp);     ///< 删除节点
            } else {
                pNode = pNode->pParent;
                cliDeleteOneNode(pNodeTmp);     ///< 删除节点
            }
        }
    }

    cliDeleteOneNode(pNodeIn);
lEnd:
    return;
}

/**
 * @brief   释放节点中的指定name子节点
 * @param   node, 节点
 * @param   name, 名称
 * @warning 节点中允许重名，该函数删除节点下所有名称符合要求的子节点
 */
void cliNodeDeleteChild(Node_s *pParent, S8 *pName)
{
    Node_s *pChild = pParent->pChild;
    Node_s *pTmp = NULL;

    while (pChild) {
        pTmp = pChild;
        pChild = pChild->pNext;
        if (cliStrCmp(pTmp->pName, pName) == 0) {
            cliDeleteNode(pTmp);
        }
    }
}

/**
 * @brief  将 child 添加到 parent 的子节点中
 * @param  parent
 * @param  child
 */
void cliNodeAddChild(Node_s *pParent, Node_s *pChild)
{
    Node_s *pNode = NULL;

    if (pChild->pParent != NULL) {
        goto lEnd;
    }
    pNode = pParent->pChild;
    pChild->pParent = pParent;

    if (pNode == NULL) {
        pParent->pChild = pChild;
        goto lEnd;
    }

    while (pNode->pNext != NULL) {
        pNode = pNode->pNext;
    }
    pNode->pNext = pChild;
lEnd:
    return;
}

/**
 * @brief  创建节点
 * @param  parent  父节点
 * @param  title   标题
 * @note   Title XXXXXXXX :
 * @note   ==============
 * @return Node_t*
 */
Node_s *cliNodeCreatObject(Node_s *pParent, S8 *pName)
{
    Node_s *pNode = NULL;

    pNode = cliNewNode(pName);
    if (pNode != NULL) {
        pNode->type   = CLI_NODE_TYPE_OBJ;
        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
        }
    }
    return pNode;
}

/**
 * @brief  创建节点
 * @param  parent 父节点
 * @param  format 标题格式
 * @param  ...    标题参数
 * @return Node_t*
 */
Node_s *cliNodeCreatObjectF(Node_s *pParent, S8 *format, ...)
{
    S32 size = 0;

    cliVaList list;
    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 title[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(title, (S32)sizeof(title), format, list);
    cliVaEnd(list);

    return cliNodeCreatObject(pParent, title);
}

/**
 * @brief  隐藏节点的名称
 * @param  node
 * @note   只在 Normal 模式下不显示名称
 */
void cliNodeHideTitle(Node_s *pNode)
{
    pNode->hideName = true;
}

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeAddKeyString(Node_s *pParent, S8 *pKey, S8 *pValue)
{
    Node_s *pNode = cliNewNode(pKey);
    S32 cliLen    = 0;

    if (pNode != NULL) {
        pNode->pValue  = cliNewString(pValue);
        pNode->type   = CLI_NODE_TYPE_KV;
        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
            cliLen = (S32)cliStrLen(pKey);
            pParent->maxLengthInSubK = MAX(pParent->maxLengthInSubK, cliLen);
        }
    }
    return;
}

/**
 * @brief  添加Value内容
 * @param  parent 父节点
 * @param  value 添加内容
 */
void cliNodeAddStringValue(Node_s *pParent, S8 *pValue)
{
    Node_s *pNode = cliNewNode(NULL);

    if (pNode != NULL) {
        pNode->pValue  = cliNewString(pValue);
        pNode->type   = CLI_NODE_TYPE_VALUE;
        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
        }
    }
    return;
}

/**
 * @brief  添加 Key - Value,并按照len长度进行换行
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeAddKeyStringCRLF(Node_s *pParent, U32 alignLen, S8 *pKey, S8 *pValue)
{
    Node_s *pNode = NULL;
    S32 cliLen    = 0;

    pNode = cliNewNode(pKey);
    if (pNode != NULL) {
        pNode->pValue    = cliNewString(pValue);
        pNode->type     = CLI_NODE_TYPE_KV;
        pNode->alignLen = alignLen;
        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
            cliLen = (S32)cliStrLen(pKey);
            pParent->maxLengthInSubK = MAX(pParent->maxLengthInSubK, cliLen);
        }
    }
    return;
}

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = 12345
 */
void cliNodeAddKeyValue(Node_s *pParent, S8 *k, S32 v)
{
    cliNodeAddKeyStringF(pParent, k, "%d", v);
    return;
}

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddKeyStringF(Node_s *pParent, S8 *pKey, S8 *format, ...)
{
    S32 size = 0;
    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 v[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(v, (S32)sizeof(v), format, list);
    cliVaEnd(list);

    cliNodeAddKeyString(pParent, pKey, v);
    return;
}

/**
 * @brief  无需key直接添加Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddStringVal(Node_s *pParent, S8 *format, ...)
{
    S32 size = 0;
    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 v[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(v, (S32)sizeof(v), format, list);
    cliVaEnd(list);

    cliNodeAddStringValue(pParent, v);
    return;
}

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddKeyStringFCRLF(Node_s *pParent, U32 alignLen, S8 *pKey, S8 *format, ...)
{
    S32 size = 0;
    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 v[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(v, (S32)sizeof(v), format, list);
    cliVaEnd(list);

    cliNodeAddKeyStringCRLF(pParent, alignLen, pKey, v);
    return;
}

/**
 * @brief  修改父节点下所有名称为 Key 的KV一级子节点的 Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeModifyKeyString(Node_s *pParent, S8 *pKey, S8 *pValue)
{
    Node_s *pNode = NULL;
    if (pParent != NULL && pParent->pChild != NULL) {
        pNode = pParent->pChild;
        while (pNode != NULL) {
            if (cliStrCmp(pNode->pName, pKey) == 0 && (pNode->type == CLI_NODE_TYPE_KV)) {
                cliFree(pNode->pValue);
                pNode->pValue = cliNewString(pValue);
                break;
            }
            pNode = pNode->pNext;
        }
    }
}

/**
 * @brief  修改父节点下所有名称为 Key 的KV一级子节点的 Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeModifyKeyStringF(Node_s *pParent, S8 *pKey, S8 *format, ...)
{
    S32 size = 0;
    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 v[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(v, (S32)sizeof(v), format, list);
    cliVaEnd(list);

    cliNodeModifyKeyString(pParent, pKey, v);
}

/**
 * @brief   查找同名表格是否存在
 * @param   parent, 父节点
 * @param   name, 表格名称
 * @return  bool
 */
Bool cliNodeTableExist(Node_s *pParent, S8 *name)
{
    Node_s *pNode = NULL;
    if (pParent != NULL) {        ///< 查找同名表格是否已经存在
        pNode = pParent->pChild;
        while (pNode != NULL) {
            if ((pNode->pName != NULL) && cliStrCmp(pNode->pName, name) == 0) {
                return true;
            }
            pNode = pNode->pNext;
        }
    }
    return false;
}

/**
 * @brief  创建表格,如果同名表格已存在,直接返回
 * @param  parent   父节点
 * @param  name     表格名
 * @param  title    表头
 * @note    ----------------------------------------------------------------------
 * @note    Title1  Title2  Title3  Title4  Title5  Title6  Title7  Title8  Title9
 * @note    ----------------------------------------------------------------------
 * @return Table_t*
 */
Node_s *cliNodeCreatTable(Node_s *pParent, S8 *name, S8 *titles[],
        Note_s *pNotes, S32 noteSize)
{
    U8     colCnt = 0;
    S32    i      = 0;
    Node_s *pNode = NULL;

    ///< 入参检查
    if (pParent != NULL) {        ///< 查找同名表格是否已经存在
        pNode = pParent->pChild;

        while (pNode) {
            if (pNode->pName && cliStrCmp(pNode->pName, name) == 0) {
                goto lEnd;
            }
            pNode = pNode->pNext;
        }
    }

    pNode = cliNewNode(name);
    if (pNode != NULL) {
        pNode->type   = CLI_NODE_TYPE_TAB;
        pNode->pTable  = cliMalloc((S32)sizeof(*pNode->pTable), 0);
        if (NULL == pNode->pTable) {
            CLI_LOG_ERROR("pNode->pTable alloc fail.");
            goto lEnd;
        }

        while (titles[colCnt] != NULL) {
            colCnt++;
        }
        pNode->pTable->colCount = colCnt;

        ///< +1 for NULL at end
        pNode->pTable->titles = cliMalloc((S32)sizeof(*pNode->pTable->titles) * (colCnt + 1), 0);
        if (NULL == pNode->pTable->titles) {
            CLI_LOG_ERROR("titles alloc failed.");
            cliFree(pNode->pTable);
        }

        for (i = 0; i < colCnt; i++) {
            pNode->pTable->titles[i] = cliNewString(titles[i]);
        }
        pNode->pTable->titles[colCnt] = NULL;

        if (pNotes != NULL) {
            cliNodeAddTableNotes(pNode->pTable, pNotes, noteSize);
        }

        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
        }
    }
lEnd:
    return pNode;
}

/**
 * @brief  创建表格的一行
 * @param  table   表格节点
 * @note   Value1  Value2  Value3  Value4  Value5  Value6  Value7  Value8  Value9
 * @return TableRow_t*
 */
TableRow_s *cliNodeCreatTableRow(Node_s *pNode)
{
    Table_s        *pTable        = NULL;
    TableRowList_s *pTableRowList = NULL;
    TableRow_s     *pRowRet       = NULL;
    TableRowList_s **ppTail       = NULL;

    ///< 入参检查
    if (pNode == NULL || pNode->pTable == NULL) {
        CLI_LOG_ERROR("input para is null.");
        goto lEnd;
    }
    pTable = pNode->pTable;

    pTableRowList = cliMalloc((S32)sizeof(*pTableRowList), 0);
    if (NULL == pTableRowList) {
        CLI_LOG_ERROR("pTableRowList alloc fail.");
        goto lEnd;
    }
    pTableRowList->row.count           = pTable->colCount;
    pTableRowList->row.pStrings         = cliMalloc((S32)sizeof(*pTableRowList->row.pStrings)
                                            * (pTableRowList->row.count + 1), 0); ///< +1 for NULL at end

    for (ppTail = &pTable->pRowList; *ppTail != NULL; ppTail = &((*ppTail)->pNext)) {
        ///< nothine to do
    }
    *ppTail = pTableRowList;
    pRowRet = &pTableRowList->row;
lEnd:
    return pRowRet;
}

/**
 * @brief  表格行内添加一个元素
 * @param  tableRow 表格行
 * @param  str      元素
 */
void cliTableRowAddString(TableRow_s *pTableRow, S8 *pStr)
{
    S32 i = 0;

    if (pTableRow == NULL) {
        CLI_LOG_ERROR("input para is null.");
        goto lEnd;
    }
    for (i = 0; i < pTableRow->count; i++) {
        if (pTableRow->pStrings[i] == NULL) {
            pTableRow->pStrings[i] = cliNewString(pStr);
            break;
        }
    }
lEnd:
    return;
}

/**
 * @brief  表格行内添加一个元素
 * @param  tableRow 表格行
 * @param  format   元素格式
 * @param  ...      元素参数
 */
void cliTableRowAddStringF(TableRow_s *pTableRow, S8 *format, ...)
{
    S32 size = 0;

    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 str[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(str, (S32)sizeof(str), format, list);
    cliVaEnd(list);

    cliTableRowAddString(pTableRow, str);
}

/**
 * @brief  表内添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliTableRowAddBuff(TableRow_s *pTableRow, U8 *v, S32 len)
{
    U8 buf[len];
    ///< S8 *pTmp = NULL;

    (void)cliMemCpy(buf, v, len);

    ///< S8 *tmp = fixString(buf, len);
    cliTableRowAddStringF(pTableRow, "%.*s", len, buf);
}

/**
 * @brief  表格添加备注
 * @param  table  表格节点
 * @param  size   备注个数
 */
void cliNodeAddTableNotes(Table_s *pTable, Note_s *pNotes, S32 size)
{
    S32 i;
    for (i = 0; i < size; i++) {
        cliNodeAddTableNote(pTable, pNotes[i].pKey, pNotes[i].pValue);
    }
}

/**
 * @brief  表格添加备注
 * @param  table  表格节点
 * @param  k      备注 key
 * @param  v      备注 value
 */
void cliNodeAddTableNote(Table_s *pTable, S8 *pKey, S8 *pValue)
{
    NoteList_s *pNoteList = NULL;
    NoteList_s **ppTail   = NULL;

    pNoteList = cliMalloc((S32)sizeof(*pNoteList), 0);
    if (NULL == pNoteList) {
        CLI_LOG_ERROR("pNoteList alloc fail.");
        goto lEnd;
    }
    pNoteList->note.pKey        = cliNewString(pKey);
    pNoteList->note.pValue      = cliNewString(pValue);

    for (ppTail = &pTable->pNoteList; *ppTail != NULL; ppTail = &((*ppTail)->pNext)) {
        ///< nothine to do
    }
    *ppTail = pNoteList;
lEnd:
    return;
}

/**
 * @brief  创建数组
 * @param  parent 父节点
 * @param  name   数组名
 * @return Node_t*
 */
Node_s *cliNodeCreatArray(Node_s *pParent, S8 *pName)
{
    Node_s *pNode = cliNewNode(pName);
    if (pNode != NULL) {
        pNode->type   = CLI_NODE_TYPE_ARR;
        if (pParent != NULL) {
            cliNodeAddChild(pParent, pNode);
        }
    }
    return pNode;
}

/**
 * @brief  同 nodeCreatArray, name可格式化
 * @param  parent 父节点
 * @param  format 格式化字符串
 * @param  ...    格式化参数
 * @return Node_t*
 */
Node_s *cliNodeCreatArrayF(Node_s *pParent, S8 *format, ...)
{
    S32 size = 0;
    Node_s *pNode = NULL;
    cliVaList list;

    cliVaStart(list, format);
    size = cliVSNPrintf(NULL, 0, format, list);
    cliVaEnd(list);

    S8 str[size + 1];

    cliVaStart(list, format);
    (void)cliVSNPrintf(str, (S32)sizeof(str), format, list);
    cliVaEnd(list);

    pNode = cliNodeCreatArray(pParent, str);
    if (NULL == pNode) {
        CLI_LOG_ERROR("pNode create fail.");
    }

    return pNode;
}

/**
 * 标准输出接口
 */
/**
 * @brief  输出连续相同的字符
 * @param  c
 * @param  count
 */
static S8 *cliGetNCharString(S8 c, S32 count)
{
    ///< 长度判断
    if (count > STR_MAX_LEN) {
        count = STR_MAX_LEN;
    }
    (void)cliMemSet(sgStrBuf, c, count);
    sgStrBuf[count] = '\0';

    return sgStrBuf;
}

/**
 * @brief  普通方式输出：打印标题
 * @param  [in] sessionId   session ID
 * @param  [in] indent      缩进
 * @param  [in] pNode       显示的节点
 * @note  Title :
 * @note  =====
 * @note
 */
static void cliShowNormalTitle(U32 sessionId, S32 indent, Node_s *pNode)
{
    S32 len = 0;
    S8 *pStr = NULL;

    if (pNode->hideName == false) {
        len = (S32)cliStrLen(pNode->pName);
        cliSessionPrintf(sessionId, "%*s%s:\n", indent * INDENT_LENGTH, "", pNode->pName);
        pStr = cliGetNCharString('-', len);
        if (NULL == pStr) {
            CLI_LOG_ERROR("pStr is NULL.");
        }
        cliSessionPrintf(sessionId, "%*s%s\n", indent * INDENT_LENGTH, "", pStr);
    }
}

/**
 * @brief  计算table每列最宽的字符数
 * @param  table    表格
 * @param  width    [out] 宽度列表
 * @param  size     宽度列表大小
 */
static void cliFillTableColWidth(Table_s *pTable, S32 *pWidth, S32 size)
{
    S32            i              = 0;
    TableRowList_s *pTableRowList = NULL;
    S32 cliLen = 0;

    for (i = 0; i < size && i < pTable->colCount; i++) {
        pWidth[i] = (S32)cliStrLen(pTable->titles[i]);
    }
    for (pTableRowList = pTable->pRowList; pTableRowList != NULL; pTableRowList = pTableRowList->pNext) {
        for (i = 0; (i < pTableRowList->row.count && i < size && pTableRowList->row.pStrings[i] != NULL); i++) {
            cliLen = (S32)cliStrLen(pTableRowList->row.pStrings[i]);
            pWidth[i] = MAX(pWidth[i], cliLen);
        }
    }
}

/**
 * @brief  普通方式输出表格
 * @param  node   节点
 * @note   Title :
 * @note       +------------------------------------------------+
 * @note       |Title1|Title2|Title3|Title4|Title5|Title6|Title7|
 * @note       +------+------+------+------+------+------+------+
 * @note       |Value1|Value2|Value3|Value4|Value5|Value6|Value7|
 * @note       |ValueA|ValueB|ValueC|ValueD|ValueE|ValueF|ValueG|
 * @note       +------------------------------------------------+
 * @note       Note1=Explanation1 | Note2=Explanation2 | Note3=Explanation3 |
 * @note       Note4=Explanation4 |
 */
static void cliShowNormalTable(U32 sessionId, S32 indent, Node_s *pNode)
{
    S32            tmpLen       = 0;
    S32            size         = 0;
    S32            i            = 0;
    S32            colMaxCount  = 0;
    S32            curLineCount = 0;
    Table_s        *pTable      = pNode->pTable;
    S32            count        = pTable->colCount;
    TableRowList_s *pRowList    = NULL;
    S32            len[count];
    S8 *   val        = NULL;
    NoteList_s *pNoteList = NULL;
    S8 *   key        = NULL;
    S8 *   value      = NULL;
    S32        tmpCount   = 0;

    cliFillTableColWidth(pTable, len, count);

    if (pNode->hideName == false) {
        ///< <INDENT> Name :
        tmpLen = (S32)cliStrLen(pNode->pName);
        cliSessionPrintf(sessionId, "%*s%s:\n",indent * INDENT_LENGTH, "", pNode->pName);
        cliSessionPrintf(sessionId, "%*s%s\n", indent * INDENT_LENGTH, "", cliGetNCharString('-', tmpLen));
    }
    indent ++;
    for (i = 0; i < count; i++) {
        size += len[i];
    }

    ///< <INDENT> +------------------------------------------------+
    cliSessionPrintf(sessionId, "%*s+%s+\n", indent * INDENT_LENGTH, "",
           cliGetNCharString('-', size + count * 2 - 2));

    ///< <INDENT> |Title1|Title2|Title3|Title4|Title5|Title6|Title7|
    cliSessionPrintf(sessionId, "%*s|", indent * INDENT_LENGTH, "");
    for (i = 0; i < count; i++) {
        cliSessionPrintf(sessionId, "%-*s", len[i] + (i + 1 == count ? 0 : 1), pTable->titles[i]);
        cliSessionPrintf(sessionId, "|");
    }
    cliSessionPrintf(sessionId, "\n");

    ///< <INDENT> +------+------+------+------+------+------+------+
    cliSessionPrintf(sessionId, "%*s+", indent * INDENT_LENGTH, "");
    for (i = 0; i < count; i++) {
        cliSessionPrintf(sessionId, "%s", cliGetNCharString('-', len[i] + (i + 1 == count ? 0 : 1)));
        cliSessionPrintf(sessionId, "+");
    }
    cliSessionPrintf(sessionId, "\n");

    ///< <INDENT> |Value1|Value2|Value3|Value4|Value5|Value6|Value7|
    for (pRowList = pTable->pRowList; pRowList != NULL; pRowList = pRowList->pNext) {
        cliSessionPrintf(sessionId, "%*s|", indent * INDENT_LENGTH, "");
        for (i = 0; i < pRowList->row.count && pRowList->row.pStrings[i] != NULL; i++) {
            val = pRowList->row.pStrings[i];
            cliSessionPrintf(sessionId, "%-*s", len[i] + (i + 1 == pRowList->row.count ? 0 : 1), val);
            cliSessionPrintf(sessionId, "|");
        }
        cliSessionPrintf(sessionId, "\n");
    }

    ///< <INDENT> +------------------------------------------------+
    cliSessionPrintf(sessionId, "%*s+%s+\n", indent * INDENT_LENGTH, "",
           cliGetNCharString('-', size + count * 2 - 2));

#if 0
    colMaxCount = cliTerminalColCount();
    if (colMaxCount <= 0) {
        colMaxCount = 80;
    }
#endif
    colMaxCount = 800;

    ///< <INDENT> Note1=Explanation1 | Note2=Explanation2 | Note3=Explanation3 |
    cliSessionPrintf(sessionId, "%*s", indent * INDENT_LENGTH, "");
    for (pNoteList = pTable->pNoteList; pNoteList != NULL; pNoteList = pNoteList->pNext) {
        key   = pNoteList->note.pKey;
        value = pNoteList->note.pValue;
        tmpCount = (S32)cliStrLen(key) + (S32)cliStrLen(value) + 4;                ///< 4: "=" + " | "
        if (curLineCount + tmpCount + 10 + indent * INDENT_LENGTH > colMaxCount) { ///< 10: 不占满屏幕，右边留10字符
            curLineCount = 0;
            cliSessionPrintf(sessionId, "\n%*s", indent * INDENT_LENGTH, "");
        }
        curLineCount += tmpCount;

        cliSessionPrintf(sessionId, "%s", key);
        cliSessionPrintf(sessionId, ":");
        cliSessionPrintf(sessionId, "%s", value);
        if (pNoteList->pNext != NULL) { ///< 最后一个字段，不打印 |
            cliSessionPrintf(sessionId, " | ");
        }
    }

    cliSessionPrintf(sessionId, "\n");
}

/**
 * @brief      普通模式显示key-value键值对
 * @param      [in]  sessionId
 * @param      [in]  indent
 * @param      [in]  pNode
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.12  xiezhj
 */
static void cliShowNormalKV(U32 sessionId, S32 indent, Node_s *pNode)
{
    S32       offset    = 0;
    S8        *pTmp     = NULL;
    S8        str[STR_MAX_LEN] = { 0 };

    offset = pNode->pParent != NULL ? pNode->pParent->maxLengthInSubK : 0;
    cliSessionPrintf(sessionId, "%*s%-*s", indent * INDENT_LENGTH, "", offset, pNode->pName);
    cliSessionPrintf(sessionId, " = ");

    ///< 输出对齐
    if (pNode->alignLen != 0 && pNode->alignLen != INVALID_CODE_U32) {
        pTmp = pNode->pValue;
        while (cliStrLen(pTmp) > pNode->alignLen) {
            (void)cliSNPrintf(str, (S32)sizeof(str), "%.*s", pNode->alignLen, pTmp);

            ///< 添加换行符以及空格,保证换行之后与第一行对齐
            ///< indent * INDENT_INDEX为树状层次间的偏移,offset为父节点下所有KV队,key的最大值
            cliSessionPrintf(sessionId, "%s\n%*s", str, (indent * INDENT_LENGTH + offset + cliStrLen(" = ")), "");
            pTmp += pNode->alignLen;
        }
        cliSessionPrintf(sessionId, "%s", pTmp);     ///< 打印剩余字符串
    } else {
        cliSessionPrintf(sessionId, "%s", pNode->pValue);
    }

    cliSessionPrintf(sessionId, "\n");
}

/**
 * @brief      普通模式显示value内容
 * @param      [in]  sessionId
 * @param      [in]  pNode
 * @return     void
 * @warning
 */
static void cliShowNormalValue(U32 sessionId, S32 indent, Node_s *pNode)
{
    cliSessionPrintf(sessionId, "%*s%s\n", indent * INDENT_LENGTH, "", pNode->pValue);
}

/**
 * @brief      普通格式显示节点信息
 * @param      [in] sessionId  session Id
 * @param      [in] pNode      要显示的节点
 * @return     void
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliShowNormal(U32 sessionId, Node_s *pNode)
{
    S32 indent = 0;     ///< 控制缩进

    ///< 循环树状结构，打印
    while (pNode != NULL) {
        switch (pNode->type) {
        case CLI_NODE_TYPE_ARR:
        case CLI_NODE_TYPE_OBJ:
            cliShowNormalTitle(sessionId, indent, pNode);
            break;
        case CLI_NODE_TYPE_TAB:
            if (pNode->pTable != NULL) {
                cliShowNormalTable(sessionId, indent, pNode);
            }
            break;
        case CLI_NODE_TYPE_KV:
            cliShowNormalKV(sessionId, indent, pNode);
            break;
        case CLI_NODE_TYPE_VALUE:
            cliShowNormalValue(sessionId, indent, pNode);
            break;
        default:
            CLI_LOG_ERROR("node type not support.");
            cliSessionPrintf(sessionId, "node type not support.");
        }

        if (pNode->pChild != NULL) {            ///< 遍历子节点
            if (false == pNode->hideName) {
                indent++;
            }
            pNode = pNode->pChild;
        } else if (pNode->pNext != NULL) {      ///< 遍历兄弟节点
            pNode = pNode->pNext;
        } else {                                ///< 上行查找叔父节点
            while (pNode != NULL) {
                pNode = pNode->pParent;
                if ((pNode != NULL) && (false == pNode->hideName)) {
                    indent--;
                }
                if ((pNode != NULL) && (pNode->pNext != NULL)) {
                    pNode = pNode->pNext;
                    break;
                }
            }
        }
    }
}

/**
 * @brief      函数说明
 * @param      [in]
 * @param      [out]
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliNodeShowNormal(U32 sessionId, Node_s *pParent)
{
    cliShowNormal(sessionId, pParent);
}
