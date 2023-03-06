#ifndef APPLICATION_H
#define APPLICATION_H

#include <QObject>

#include "libtree.h"
#include "documents.h"

class Application : public QObject, public TDocStd_Application {
    Q_OBJECT
public:
    ~Application();
    static const Handle(Application)& instance();
    Handle(Document) newDocument(Document::Format docFormat = Document::Format::Binary);
    int documentCount() const;

public: //  from TDocStd_Application
    void NewDocument(
            const TCollection_ExtendedString& format,
            opencascade::handle<TDocStd_Document>& outDoc) override;

    void InitDocument(const opencascade::handle<TDocStd_Document>& doc) const override;


private:
     Application(){}
};
#endif // APPLICATION_H
