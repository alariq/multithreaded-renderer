#pragma once

/* Extracted and modified from VulkanMemoryAllocator */

#include<cstring> // memcpy
#include<cstdio>
#include<cassert>
#include<cstdint>
#include<vector>

#define CLASS_NO_COPY(Klass)\
    Klass& operator=(const Klass&) = delete;\
    Klass(const Klass&) = delete;

#define ASSERT(x) assert(x)

static inline void Uint32ToStr(char* outStr, size_t strLen, uint32_t num)
{
    snprintf(outStr, strLen, "%u", static_cast<unsigned int>(num));
}
static inline void Uint64ToStr(char* outStr, size_t strLen, uint64_t num)
{
    snprintf(outStr, strLen, "%llu", static_cast<unsigned long long>(num));
}
static inline void PtrToStr(char* outStr, size_t strLen, const void* ptr)
{
    snprintf(outStr, strLen, "%p", ptr);
}

class StringBuilder
{
public:
    StringBuilder() { }
    size_t GetLength() const { return m_Data.size(); }
    const char* GetData() const { return m_Data.data(); }

    void Add(char ch) { m_Data.push_back(ch); }
    void Add(const char* pStr);
    void AddNewLine() { Add('\n'); }
    void AddNumber(uint32_t num);
    void AddNumber(uint64_t num);
    void AddPointer(const void* ptr);

private:
    std::vector<char> m_Data;
};

void StringBuilder::Add(const char* pStr)
{
    const size_t strLen = strlen(pStr);
    if(strLen > 0)
    {
        const size_t oldCount = m_Data.size();
        m_Data.resize(oldCount + strLen);
        memcpy(m_Data.data() + oldCount, pStr, strLen);
    }
}

void StringBuilder::AddNumber(uint32_t num)
{
    char buf[11];
    Uint32ToStr(buf, sizeof(buf), num);
    Add(buf);
}

void StringBuilder::AddNumber(uint64_t num)
{
    char buf[21];
    Uint64ToStr(buf, sizeof(buf), num);
    Add(buf);
}

void StringBuilder::AddPointer(const void* ptr)
{
    char buf[21];
    PtrToStr(buf, sizeof(buf), ptr);
    Add(buf);
}

class JsonWriter
{
    CLASS_NO_COPY(JsonWriter)
public:
    JsonWriter(StringBuilder& sb);
    ~JsonWriter();

    void BeginObject(bool singleLine = false);
    void EndObject();
    
    void BeginArray(bool singleLine = false);
    void EndArray();
    
    void WriteString(const char* pStr);
    void BeginString(const char* pStr = nullptr);
    void ContinueString(const char* pStr);
    void ContinueString(uint32_t n);
    void ContinueString(uint64_t n);
    void ContinueString_Pointer(const void* ptr);
    void EndString(const char* pStr = nullptr);
    
    void WriteNumber(uint32_t n);
    void WriteNumber(uint64_t n);
    void WriteBool(bool b);
    void WriteNull();

private:
    static const char* const INDENT;

    enum COLLECTION_TYPE
    {
        COLLECTION_TYPE_OBJECT,
        COLLECTION_TYPE_ARRAY,
    };
    struct StackItem
    {
        COLLECTION_TYPE type;
        uint32_t valueCount;
        bool singleLineMode;
    };

    StringBuilder& m_SB;
    std::vector<StackItem> m_Stack;
    bool m_InsideString;

    void BeginValue(bool isString);
    void WriteIndent(bool oneLess = false);
};

const char* const JsonWriter::INDENT = "  ";

JsonWriter::JsonWriter(StringBuilder& sb) :
    m_SB(sb),
    m_Stack(),
    m_InsideString(false)
{
}

JsonWriter::~JsonWriter()
{
    ASSERT(!m_InsideString);
    ASSERT(m_Stack.empty());
}

void JsonWriter::BeginObject(bool singleLine)
{
    ASSERT(!m_InsideString);

    BeginValue(false);
    m_SB.Add('{');

    StackItem item;
    item.type = COLLECTION_TYPE_OBJECT;
    item.valueCount = 0;
    item.singleLineMode = singleLine;
    m_Stack.push_back(item);
}

void JsonWriter::EndObject()
{
    ASSERT(!m_InsideString);

    WriteIndent(true);
    m_SB.Add('}');

    ASSERT(!m_Stack.empty() && m_Stack.back().type == COLLECTION_TYPE_OBJECT);
    m_Stack.pop_back();
}

void JsonWriter::BeginArray(bool singleLine)
{
    ASSERT(!m_InsideString);

    BeginValue(false);
    m_SB.Add('[');

    StackItem item;
    item.type = COLLECTION_TYPE_ARRAY;
    item.valueCount = 0;
    item.singleLineMode = singleLine;
    m_Stack.push_back(item);
}

void JsonWriter::EndArray()
{
    ASSERT(!m_InsideString);

    WriteIndent(true);
    m_SB.Add(']');

    ASSERT(!m_Stack.empty() && m_Stack.back().type == COLLECTION_TYPE_ARRAY);
    m_Stack.pop_back();
}

void JsonWriter::WriteString(const char* pStr)
{
    BeginString(pStr);
    EndString();
}

void JsonWriter::BeginString(const char* pStr)
{
    ASSERT(!m_InsideString);

    BeginValue(true);
    m_SB.Add('"');
    m_InsideString = true;
    if(pStr != NULL && pStr[0] != '\0')
    {
        ContinueString(pStr);
    }
}

void JsonWriter::ContinueString(const char* pStr)
{
    ASSERT(m_InsideString);

    const size_t strLen = strlen(pStr);
    for(size_t i = 0; i < strLen; ++i)
    {
        char ch = pStr[i];
        if(ch == '\\')
        {
            m_SB.Add("\\\\");
        }
        else if(ch == '"')
        {
            m_SB.Add("\\\"");
        }
        else if(ch >= 32)
        {
            m_SB.Add(ch);
        }
        else switch(ch)
        {
        case '\b':
            m_SB.Add("\\b");
            break;
        case '\f':
            m_SB.Add("\\f");
            break;
        case '\n':
            m_SB.Add("\\n");
            break;
        case '\r':
            m_SB.Add("\\r");
            break;
        case '\t':
            m_SB.Add("\\t");
            break;
        default:
            ASSERT(0 && "Character not currently supported.");
            break;
        }
    }
}

void JsonWriter::ContinueString(uint32_t n)
{
    ASSERT(m_InsideString);
    m_SB.AddNumber(n);
}

void JsonWriter::ContinueString(uint64_t n)
{
    ASSERT(m_InsideString);
    m_SB.AddNumber(n);
}

void JsonWriter::ContinueString_Pointer(const void* ptr)
{
    ASSERT(m_InsideString);
    m_SB.AddPointer(ptr);
}

void JsonWriter::EndString(const char* pStr)
{
    ASSERT(m_InsideString);
    if(pStr != NULL && pStr[0] != '\0')
    {
        ContinueString(pStr);
    }
    m_SB.Add('"');
    m_InsideString = false;
}

void JsonWriter::WriteNumber(uint32_t n)
{
    ASSERT(!m_InsideString);
    BeginValue(false);
    m_SB.AddNumber(n);
}

void JsonWriter::WriteNumber(uint64_t n)
{
    ASSERT(!m_InsideString);
    BeginValue(false);
    m_SB.AddNumber(n);
}

void JsonWriter::WriteBool(bool b)
{
    ASSERT(!m_InsideString);
    BeginValue(false);
    m_SB.Add(b ? "true" : "false");
}

void JsonWriter::WriteNull()
{
    ASSERT(!m_InsideString);
    BeginValue(false);
    m_SB.Add("null");
}

void JsonWriter::BeginValue(bool isString)
{
    if(!m_Stack.empty())
    {
        StackItem& currItem = m_Stack.back();
        if(currItem.type == COLLECTION_TYPE_OBJECT &&
            currItem.valueCount % 2 == 0)
        {
            ASSERT(isString);
        }

        if(currItem.type == COLLECTION_TYPE_OBJECT &&
            currItem.valueCount % 2 != 0)
        {
            m_SB.Add(": ");
        }
        else if(currItem.valueCount > 0)
        {
            m_SB.Add(", ");
            WriteIndent();
        }
        else
        {
            WriteIndent();
        }
        ++currItem.valueCount;
    }
}

void JsonWriter::WriteIndent(bool oneLess)
{
    if(!m_Stack.empty() && !m_Stack.back().singleLineMode)
    {
        m_SB.AddNewLine();
        
        size_t count = m_Stack.size();
        if(count > 0 && oneLess)
        {
            --count;
        }
        for(size_t i = 0; i < count; ++i)
        {
            m_SB.Add(INDENT);
        }
    }
}

