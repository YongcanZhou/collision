#ifndef DOCUMENTS_H
#define DOCUMENTS_H

#include <QObject>
#include "libtree.h"

class Document : public QObject, public TDocStd_Document
{
    Q_OBJECT
public:
    Document();
    enum class Format { Binary, Xml };
    static const char* toNameFormat(Format format);
    void setName(const QString& name_){name=name_;}
private:



public:
    Assemly_Tree docTree;
    QString name;
};


class DocumentAssemNode{

};

#endif // DOCUMENTS_H
