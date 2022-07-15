
#ifndef BUTTON_FLAT_H
#define BUTTON_FLAT_H
#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtGui/QBrush>
#include <QtWidgets/QWidget>
#include <QPainter>
#include <QtCore/QSignalBlocker>
#include <QtGui/QCursor>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QAction>
class QAction;

namespace Ui
{
// 自定义 checkbutton
class ButtonFlat : public QWidget {
    Q_OBJECT
public:
    ButtonFlat(QWidget* parent = nullptr);

    bool isCheckable() const;
    void setCheckable(bool on);

    bool isChecked() const;
    void setChecked(bool on);

    const QIcon& icon() const;
    void setIcon(const QIcon& icon);

    const QSize& iconSize() const;
    void setIconSize(const QSize& size);

    QAction* defaultAction() const;
    void setDefaultAction(QAction* action);

    const QBrush& hoverBrush() const;
    void setHoverBrush(const QBrush& brush);

    const QBrush& checkedBrush() const;
    void setCheckedBrush(const QBrush& brush);

    const QBrush& backgroundBrush() const;
    void setBackgroundBrush(const QBrush& brush);

    const QVariant& data() const { return m_data; }
    void setData(const QVariant& var) { m_data = var; }

signals:
    void clicked();
    void checked(bool on);

protected:
    void paintEvent(QPaintEvent* event) override;
    void enterEvent(QEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void syncToAction();

    bool m_isMouseHover = false;
    bool m_isCheckable = false;
    bool m_isChecked = false;
    QIcon m_icon;
    QSize m_iconSize;
    QBrush m_hoverBrush;
    QBrush m_checkedBrush;
    QAction* m_defaultAction = nullptr;
    QVariant m_data;
};

} // namespace Mayo
#endif // BUTTON_FLAT_H