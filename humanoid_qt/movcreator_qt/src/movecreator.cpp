#include "../include/movcreator_qt/movecreator.h"

namespace movcreator_qt {
MoveCreator::MoveCreator(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MoveCreator),
    qnode(new QNode(argc,argv))
{
    ui->setupUi(this);
    signalReceived = false;
    lastPoseIndex = -1;

    //Configurando o QListWidget
    ui->poseListWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    ui->poseListWidget->setFlow(QListView::LeftToRight);
    ui->poseListWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);

    ui->poseListWidget->setAcceptDrops(true);
    ui->poseListWidget->setDragEnabled(true);

    ui->poseListWidget->setDropIndicatorShown(true);
    ui->poseListWidget->setDragDropMode(QAbstractItemView::InternalMove);

    createConnections();
    connectToROS();

    setBasicLayout();
    getActiveMotors();

    int count = 0;
    while(activeMotors.empty()) //esperando a resposta do ros
    {
        if( count > 25000)
        {
            ui->consoleLabel->setText("Only offline available");
            break;
        }
        count++;
    }
    //Inicialização dos valores de scan
    for(int i = 0; i < activeMotors.size(); i++)
        scanValues.push_back(0);
    setScanValues(scanValues);

    //TODO:Verificar se não é melhor deixar em outro local
    ///////////////////////////////////////////////////////////////////////////////////////
    //Iniciando com todos os motores on
    for(unsigned int i = 0; i < activeMotors.size();i++)
       motorOn.push_back(true);
    ///////////////////////////////////////////////////////////////////////////////////////

    createActions();
    createMenus();
    ui->poseListWidget->viewport()->installEventFilter(this);
}

MoveCreator::~MoveCreator()
{
    delete ui;
}

void MoveCreator::savePage()
{
    //TODO:Verificar o que acontece caso o arquivo já exista
    if(lastPoseIndex < 0)
    {
       ui->consoleLabel->setText("Cannot save empty page.");
    }
    else
    {
        if(fileName.isEmpty())
        {
            fileName = QFileDialog::getSaveFileName(this, tr("Save page"), "", tr("Page file(.page);"));
            fileName.append(".page");
        }

        Page page = generatePage();
        page.writeNode(fileName.toStdString());
        ui->consoleLabel->setText("Pagina salva com sucesso");
    }
}

void MoveCreator::loadPage()
{
    //TODO:Adicionar seguranças relacionadas aos motores ativos ou formato errado de page

    QString loadFileName = QFileDialog::getOpenFileName(this, tr("Load page"), "", tr(""));
    if(loadFileName.isEmpty())  //Segurança para caso o usuário feche a janela
        return;

    Page loadPage(loadFileName.toStdString());
    fileName = loadFileName;

    lastPoseIndex = -1;
    ui->poseListWidget->clear();

    for(unsigned int i = 0; i < loadPage.getPoses().size();i++)
    {
        addPose();
        QWidget *currentItem = ui->poseListWidget->itemWidget(ui->poseListWidget->item(ui->poseListWidget->currentRow()));
        PoseInterface *pose = dynamic_cast<PoseInterface*>(currentItem);
        pose->fromPose(loadPage.getPoses()[i]);
    }
}

void MoveCreator::quit()
{
    //TODO: Verificar se o usuário não deseja salvar a page
    this->close();
}

Page MoveCreator::generatePage()
{
    std::vector<Pose> poses;
    for(int i = 0; i <= lastPoseIndex; i++)
    {
        QWidget *currentItem = ui->poseListWidget->itemWidget(ui->poseListWidget->item(i));
        PoseInterface *pose = dynamic_cast<PoseInterface*>(currentItem);
        poses.push_back(pose->toPose());
    }
    Page page(poses);
    return page;
}


PoseInterface * MoveCreator::createPoseInterface()
{
    PoseInterface *pose = new PoseInterface();
    pose->setActiveMotors(activeMotors);
    pose->setStyleSheet("selection-color: rgb(114, 159, 207);");
    pose->disable();
    return pose;
}

void MoveCreator::addPose()
{
    lastPoseIndex ++;
    PoseInterface *pose = createPoseInterface();
    QListWidgetItem *item = new QListWidgetItem(ui->poseListWidget);
    item->setSizeHint(pose->size());
    ui->poseListWidget->addItem(item);
    ui->poseListWidget->setItemWidget(item, pose);

    moveToRight();
    /*if(lastPoseIndex == 0) //Esse if faz com que a primeira pose fique selecionada
    {
        moveToRight();
    }*/
}




void MoveCreator::moveToLeft()
{
    if(ui->poseListWidget->currentRow() < 0)
        return;
    else if(ui->poseListWidget->currentRow() == 0)
    {
        ui->poseListWidget->clearSelection();
        ui->poseListWidget->setCurrentRow(ui->poseListWidget->currentRow() - 1);
    }
    else
    {
        ui->poseListWidget->clearSelection();
        ui->poseListWidget->item(ui->poseListWidget->currentRow() - 1)->setSelected(true);
        ui->poseListWidget->setCurrentRow(ui->poseListWidget->currentRow() - 1);
    }
}

void MoveCreator::moveToRight()
{
    if(ui->poseListWidget->currentRow() >= lastPoseIndex)
        return;
    else
    {
        ui->poseListWidget->clearSelection();
        ui->poseListWidget->item(ui->poseListWidget->currentRow() + 1)->setSelected(true);
        ui->poseListWidget->setCurrentRow(ui->poseListWidget->currentRow() + 1);
    }
}

void MoveCreator::scanCompleted()
{
    setScanValues(qnode->getMotorsPosition());
    signalReceived = true;
}

void MoveCreator::idsScanCompleted()
{
     activeMotors = qnode->getActiveMotors();
     setupMotorLabels();
}

void MoveCreator::deletePose()
{
    //Determinando quantas poses serão excluidas
    QList<QListWidgetItem*> items = ui->poseListWidget->selectedItems();
    lastPoseIndex -= items.size();

    qDeleteAll(ui->poseListWidget->selectedItems());
    ui->poseListWidget->setCurrentRow(0);
}

void MoveCreator::scanPose()
{
    Q_EMIT requestScan();
}

void MoveCreator::setScanValues(std::vector<double> positions)
{
    for(unsigned int i = 0; i < scanButtons.size(); i ++)
    {
        scanValues[i] = positions[i] * (180.0/ M_PI); //Conversão de radianos para graus
        scanButtons[i]->setText(QString::number(scanValues[i],'g',4));
    }
}

void MoveCreator::writePose()
{
    if(lastPoseIndex >= 0)  //Segurança para quando não existir nenhuma pose
    {
        QList<QListWidgetItem*> items = ui->poseListWidget->selectedItems();
        Q_FOREACH (QListWidgetItem* item, items)  //Escreve em todas as poses selecionadas
        {
            QWidget *currentItem = ui->poseListWidget->itemWidget(item);
            PoseInterface *pose = dynamic_cast<PoseInterface*>(currentItem);
            pose->setJointPositions(scanValues);
        }
    }
}

void MoveCreator::copyPose()
{
    copiedPoses.clear();
    QList<QListWidgetItem*> items = ui->poseListWidget->selectedItems();
    Q_FOREACH (QListWidgetItem* item, items)
    {
        QWidget *currentItem = ui->poseListWidget->itemWidget(item);
        PoseInterface *pose = dynamic_cast<PoseInterface*>(currentItem);
        copiedPoses.push_back(pose->toPose());
    }
}

void MoveCreator::cutPose()
{
    copyPose();
    deletePose();
}

void MoveCreator::pastePose()
{
    if(copiedPoses.empty())
        return;
    else
    {
        if(ui->poseListWidget->selectedItems().size() > 1)
        {
            ui->poseListWidget->clearSelection();
            ui->consoleLabel->setText("Selecione o local para colar");
        }
        else //TODO:Verificar se deixa o código assim ou se modifica a função addPose para ser mais genérica e evitar repetição de código
        {
            //Se nenhuma pose estiver selecionada, será colocado na posição 0
            int currentRow = -1;
            if(ui->poseListWidget->selectedItems().size()  > 0)
                currentRow = ui->poseListWidget->row(ui->poseListWidget->selectedItems().at(0));

            for(unsigned int i = 0; i < copiedPoses.size(); i++)
            {
                lastPoseIndex ++;
                int row = currentRow + 1 + i;
                QListWidgetItem *item = new QListWidgetItem();
                PoseInterface *pose = createPoseInterface();
                pose->fromPose(copiedPoses[i]);
                item->setSizeHint(pose->size());
                ui->poseListWidget->insertItem(row,item);
                ui->poseListWidget->setItemWidget(item, pose);
                copiedPoses.clear();
            }
        }
    }
}

void MoveCreator::turnOnOffMotor()
{
    //alerta gambiarra
    //scanThenSend();


    //Código para determinar o id do motor que deve ser desligado por meio do texto do botão
    QPushButton *motorButton = (QPushButton *)sender();
    std::string buttonText = motorButton->text().toStdString();
    std::string space = " ";
    std::string numberString = buttonText.substr(buttonText.find(space),buttonText.size());
    int motorIndex = std::stoi(numberString);

    if(motorOn[motorIndex] == true) //Motor está ligado, portanto irá desligá-lo
    {
        motorOn[motorIndex] = false;
        motorButton->setStyleSheet("background-color: rgb(250, 155, 155);");
    }
    else
    {
        motorOn[motorIndex] = true;
        motorButton->setStyleSheet("background-color: lightgreen;");
    }

    setTorque();
}

void MoveCreator::turnAllOn()
{
    //alerta gambiarra
    //scanThenSend();

    for(unsigned int i = 0; i < activeMotors.size();i++)
    {
       motorOn[i] = true;
       motorButtons[i]->setStyleSheet("background-color: lightgreen;");
    }
    setTorque();
}

void MoveCreator::turnAllOff()
{
    //alerta gambiarra
    //scanThenSend();

    for(unsigned int i = 0; i < activeMotors.size();i++)
    {
       motorOn[i] = false;
       motorButtons[i]->setStyleSheet("background-color: rgb(250, 155, 155);");
    }
    setTorque();
}

void MoveCreator::saveAs()
{
    fileName.clear();
    savePage();
}

void MoveCreator::createConnections()
{
    QObject::connect(ui->allOnButton,SIGNAL(released()),this,SLOT(turnAllOn()));
    QObject::connect(ui->allOffButton,SIGNAL(released()),this,SLOT(turnAllOff()));
    QObject::connect(ui->playButton,SIGNAL(released()),this,SLOT(playPage()));
    connect(ui->poseListWidget->selectionModel(), SIGNAL(selectionChanged(const QItemSelection&,const QItemSelection&)), this, SLOT(poseSelectionChanged(const QItemSelection&,const QItemSelection&)));
}

void MoveCreator::connectToROS()
{
    qnode->init();
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(quit()));
    QObject::connect(this, SIGNAL(torqueSignal(std::vector<int>, std::vector<bool>)), qnode, SLOT(setTorque(std::vector<int>, std::vector<bool>)));
    QObject::connect(this, SIGNAL(requestScan()), qnode, SLOT(scanMotor()));
    QObject::connect(this, SIGNAL(scanActiveMotors()), qnode, SLOT(scanActiveMotors()));
    QObject::connect(qnode, SIGNAL(scanFinished()), this, SLOT(scanCompleted()));
    QObject::connect(qnode, SIGNAL(idScanFinished()), this, SLOT(idsScanCompleted()));

    QObject::connect(this, SIGNAL(playPageSignal(std::string)), qnode, SLOT(playPage(std::string)));
    QObject::connect(this, SIGNAL(goPoseSignal(std::string)), qnode, SLOT(goPose(std::string)));
}

void MoveCreator::setBasicLayout()
{
    ui->iconLabel->setPixmap(QPixmap(":/images/edrom.png"));
    ui->iconLabel->setScaledContents(true);

    ui->consoleLabel->setFrameStyle(1);
    ui->consoleLabel->setAlignment(Qt::AlignJustify);

    ui->playButton->setIcon(QIcon(QPixmap(":/images/play.png")));
    ui->playButton->setIconSize(QSize(20,20));

    ui->allOffButton->setIcon(QIcon(QPixmap(":/images/on-off.png")));
    ui->allOffButton->setIconSize(QSize(20,20));
    ui->allOffButton->setStyleSheet("background-color: rgb(250, 155, 155);");

    ui->allOnButton->setIcon(QIcon(QPixmap(":/images/on-off.png")));
    ui->allOnButton->setIconSize(QSize(20,20));
    ui->allOnButton->setStyleSheet("background-color: lightgreen;");
}

void MoveCreator::setupMotorLabels()
{

    QPushButton *motorLabel;
    QPushButton *label;
    for(unsigned int i = 0; i < activeMotors.size(); i ++)
    {
        motorLabel = new QPushButton;
        connect(motorLabel, SIGNAL(released()), this, SLOT(turnOnOffMotor()));
        motorLabel->setText(QString("Motor %1").arg((activeMotors[i])));
        motorLabel->setStyleSheet("background-color: lightgreen;");
        ui->motorVerticalLayout->addWidget(motorLabel);
        motorButtons.push_back(motorLabel);

        label = new QPushButton;
        label->setText(QString("%1").arg((0)));
        ui->scanVerticalLayout->addWidget(label);
        scanButtons.push_back(label);
    }
    motorLabel = new QPushButton;
    motorLabel->setText("Time");
    motorLabel->setDisabled(true);
    motorLabel->setStyleSheet("color: black;");
    ui->motorVerticalLayout->addWidget(motorLabel);

    motorLabel = new QPushButton;
    motorLabel->setText("Pause time");
    motorLabel->setDisabled(true);
    motorLabel->setStyleSheet("color: black;");
    ui->motorVerticalLayout->addWidget(motorLabel);
}

void MoveCreator::getActiveMotors()
{
    //TODO: Criar diálogo para determinar se o usuário quer que os motores da cabeça façam parte da page - default:não utiliza a cabeça
   // Q_EMIT scanActiveMotors();

    ////////////////////////////////temporário////////////////////////////////////////////
    activeMotors.clear();
    for(int i = 0; i < 20; i++)  //Não estamos verificando os ids reais atualmente
        activeMotors.push_back(i);
    setupMotorLabels();
    //////////////////////////////////////////////////////////////////////////////////////
}

void MoveCreator::setTorque()
{
    Q_EMIT torqueSignal(activeMotors,motorOn);
}

//Gambiarra
void MoveCreator::scanThenSend()
{
    signalReceived = false;
    requestScan();

    while(!signalReceived);
    std::vector<JointCommand> joints;
    for(unsigned int i = 0; i < activeMotors.size();i++)
    {
        JointCommand joint;
        joint.setId(activeMotors[i]);
        joint.setPosition((scanValues[i])/(180.0/ M_PI));
        joint.setTorque(-1);
        joint.setVelocity(-1);
        joint.setAcceleration(-1);
        joints.push_back(joint);
    }
    Pose pose;
    pose.setTime(0.07);
    pose.setPauseTime(0);
    pose.setJoints(joints);

    std::vector<Pose> poses;
    poses.push_back(pose);
    Page page;
    page.setPoses(poses);
    std::string path;
    path = ros::package::getPath("motor_set_control") + "/pose.page";
    page.writeNode(path);

    Q_EMIT goPoseSignal(path);
}

void MoveCreator::createActions()
{
    //File menu

    playAct  = new QAction(tr("&Play page"), this);
    playAct->setShortcuts(QKeySequence::Print);
    QObject::connect(playAct, SIGNAL(triggered()), this, SLOT(playPage()));

    goPoseAct  = new QAction(tr("&Go Pose"), this);
    goPoseAct->setShortcuts(QKeySequence::Forward);
    QObject::connect(goPoseAct, SIGNAL(triggered()), this, SLOT(goPose()));

    saveAct = new QAction(tr("&Save"), this);
    saveAct->setShortcuts(QKeySequence::Save);
    QObject::connect(saveAct, SIGNAL(triggered()), this, SLOT(savePage()));

    saveAsAct = new QAction(tr("&SaveAs"), this);
    saveAsAct->setShortcuts(QKeySequence::SaveAs);
    QObject::connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAs()));

    loadAct = new QAction(tr("&Load"), this);
    loadAct->setShortcuts(QKeySequence::Open);
    QObject::connect(loadAct, SIGNAL(triggered()), this, SLOT(loadPage()));

    quitAct = new QAction(tr("&Quit"), this);
    quitAct->setShortcut(Qt::Key_Escape);
    QObject::connect(quitAct, SIGNAL(triggered()), this, SLOT(quit()));

    //Edit menu

    addPoseAct = new QAction(tr("&New Pose"), this);
    addPoseAct->setShortcut(Qt::Key_Space);
    QObject::connect(addPoseAct, SIGNAL(triggered()), this, SLOT(addPose()));

    deletePoseAct = new QAction(tr("&Delete pose"), this);
    deletePoseAct->setShortcut(Qt::Key_Delete);
    QObject::connect(deletePoseAct, SIGNAL(triggered()), this, SLOT(deletePose()));

    scanPoseAct = new QAction(tr("&Scan pose"), this);
    scanPoseAct->setShortcut(Qt::Key_F5);;
    QObject::connect(scanPoseAct, SIGNAL(triggered()), this, SLOT(scanPose()));

    writePoseAct = new QAction(tr("&Write pose"), this);
    writePoseAct->setShortcut(Qt::Key_W);
    QObject::connect(writePoseAct, SIGNAL(triggered()), this, SLOT(writePose()));

    copyPoseAct = new QAction(tr("&Copy pose"), this);
    copyPoseAct->setShortcuts(QKeySequence::Copy);
    QObject::connect(copyPoseAct, SIGNAL(triggered()), this, SLOT(copyPose()));

    cutPoseAct = new QAction(tr("&Cut pose"), this);
    cutPoseAct->setShortcuts(QKeySequence::Cut);
    QObject::connect(cutPoseAct, SIGNAL(triggered()), this, SLOT(cutPose()));

    pastePoseAct = new QAction(tr("&Paste pose"), this);
    pastePoseAct->setShortcuts(QKeySequence::Paste);
    QObject::connect(pastePoseAct, SIGNAL(triggered()), this, SLOT(pastePose()));

    moveLeftAct = new QAction(tr("Move to left"),this);
    moveLeftAct->setShortcut(Qt::Key_Left);
    QObject::connect(moveLeftAct, SIGNAL(triggered()), this, SLOT(moveToLeft()));

    moveRightAct = new QAction(tr("Move to Right"),this);
    moveRightAct->setShortcut(Qt::Key_Right);
    QObject::connect(moveRightAct, SIGNAL(triggered()), this, SLOT(moveToRight()));

    //Help Menu
}

void MoveCreator::createMenus()
{
    menuBar()->setNativeMenuBar(false);

    fileMenu = menuBar()->addMenu(tr("File"));
    fileMenu->addAction(playAct);
    fileMenu->addAction(goPoseAct);
    fileMenu->addSeparator();
    fileMenu->addAction(saveAct);
    fileMenu->addAction(saveAsAct);
    fileMenu->addAction(loadAct);
    fileMenu->addSeparator();
    fileMenu->addAction(quitAct);

    editMenu = menuBar()->addMenu(tr("Edit"));
    editMenu->addAction(addPoseAct);
    editMenu->addAction(deletePoseAct);
    editMenu->addSeparator();
    editMenu->addAction(scanPoseAct);
    editMenu->addAction(writePoseAct);
    editMenu->addSeparator();
    editMenu->addAction(copyPoseAct);
    editMenu->addAction(cutPoseAct);
    editMenu->addAction(pastePoseAct);
    editMenu->addSeparator();
    editMenu->addAction(moveRightAct);
    editMenu->addAction(moveLeftAct);

    helpMenu = menuBar()->addMenu(tr("Help"));

}

//TODO:Verificar se não fica lento quando existem muitas poses na página
void MoveCreator::poseSelectionChanged(const QItemSelection & selected,const QItemSelection & deselected)
{
    QWidget *currentItem;
    PoseInterface *pose;
    QModelIndexList indexes = selected.indexes();

    Q_FOREACH(const QModelIndex &index, indexes)
    {
        currentItem = ui->poseListWidget->itemWidget(ui->poseListWidget->item(index.row()));
        if(currentItem != NULL)  //Segurança já que o método delete gera o sinal
        {
            pose = dynamic_cast<PoseInterface*>(currentItem);
            pose->enable();
        }
    }

    indexes = deselected.indexes();
    Q_FOREACH(const QModelIndex &index, indexes)
    {
        currentItem = ui->poseListWidget->itemWidget(ui->poseListWidget->item(index.row()));
        if(currentItem != NULL)  //Segurança já que o método delete gera o sinal
        {
            pose = dynamic_cast<PoseInterface*>(currentItem);
            pose->disable();   
        }
    }
}

bool MoveCreator::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::Paint)
    {
        QWidget *currentItem;
        PoseInterface *pose;
        for(int row = 0; row < ui->poseListWidget->count(); row++)
        {
            currentItem = ui->poseListWidget->itemWidget(ui->poseListWidget->item(row));
            if(currentItem != NULL)  //Segurança já que o método delete gera o sinal
            {
                pose = dynamic_cast<PoseInterface*>(currentItem);
                int indexNumber = ui->poseListWidget->row(ui->poseListWidget->item(row));
                pose->setPoseNumber(indexNumber);
            }
        }
    }
    return false;
}


void MoveCreator::playPage()
{
    std::string auxiliar = fileName.toStdString();
    fileName.clear();
    fileName.append(ros::package::getPath("motor_set_control").c_str());
    fileName.append( "/porao.page");
    savePage();
    Q_EMIT playPageSignal(fileName.toStdString());
    fileName.clear();
    fileName.append(auxiliar.c_str());
}

void MoveCreator::goPose()
{
    QList<QListWidgetItem*> items = ui->poseListWidget->selectedItems();
    if(items.size() > 1)
    {
        ui->consoleLabel->setText("Mais de uma pose selecionada");
        ui->poseListWidget->clearSelection();
    }
    else if(items.size() == 0)
        ui->consoleLabel->setText("Nenhuma pose selecionada");
    else
    {
        QWidget *currentItem = ui->poseListWidget->itemWidget(items[0]);
        PoseInterface *poseInterface = dynamic_cast<PoseInterface*>(currentItem);
        std::vector<Pose> pose;
        pose.push_back(poseInterface->toPose());
        Page page;
        page.setPoses(pose);

        std::string path;
        path = ros::package::getPath("motor_set_control") + "/pose.page";
        page.writeNode(path);

        Q_EMIT goPoseSignal(path);
    }
}

#ifndef QT_NO_CONTEXTMENU
void MoveCreator::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.exec(event->globalPos());
}

#endif // QT_NO_CONTEXTMENU
}
