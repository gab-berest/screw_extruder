% **************************************************************
% Programme : Identification des parametres d'un système linéaire
%
% Auteur : Alexandre Borowczyk
% Date : 28/01/15
% Version : Version Hiver 2015 v2
% **************************************************************
close all
clc

global Commande Temps fHandle signal_exp

%Get workspace variables
Workspace_var = whos;

for i = 1:length(Workspace_var)
    Name_list{i} = Workspace_var(i).name;
    Class_List{i} = Workspace_var(i).class;
end
valid_var = strcmp(Class_List,'struct');
Workspace_var = Workspace_var(valid_var);

[Selection,ok] = listdlg('PromptString','Signal de Commande',...
    'SelectionMode','single',...
    'ListString',Name_list(valid_var));

if (~ok)
    
    disp('Opération annulée')
else
    
    Temps = eval([Workspace_var(Selection).name '.time']);
    Commande = eval([Workspace_var(Selection).name '.signals.values']);
    
    [Selection,ok] = listdlg('PromptString','Signal de Sortie',...
        'SelectionMode','single',...
        'ListString',Name_list(valid_var));
    
    if (~ok)
        
        disp('Opération annulée')
        
    else
        
        Out_Time = eval([Workspace_var(Selection).name '.time']);
        Out_signals = eval([Workspace_var(Selection).name '.signals']);
        nbrOut = Out_signals.dimensions;
        titre_signal = Out_signals.label(1:end);
        index = 1;
        
        figure_handle = figure;
        for i=1:nbrOut
            
            % Affichage des Donnees
            
            subplot(nbrOut,1,i)
            plot(Out_Time, Out_signals.values(:,i))
            try
                temp = find(titre_signal(index:end) == ',',1,'first')+index-1;
                if isempty(temp)
                    temp = length(titre_signal(index:end));
                end
                titres{i} = [titre_signal(index:temp-1), sprintf(' (%i)',i)];
                title( titres{i} )
                index = temp+1;
            catch
                title('error reading signal name')
            end
            
        end
        
        prompt = {'Quel signal utiliser :'};
        dlg_title = 'Sélection signal';
        num_lines = 1;
        def = {'1'};
        answer = inputdlg(prompt,dlg_title,num_lines,def);
        
        if (~ok)
            
            disp('Opération annulée')
            
        else
            
            signal_exp = Out_signals.values(:,str2num(answer{1}))';
            
            
            % Affichage des Donnees
            figure_handle = figure;
            subplot(2,1,1)
            plot(Temps, signal_exp)
            title(titres{str2num(answer{1})})
            subplot(2,1,2)
            plot(Temps, Commande)
            title('Commande')
            
            
            
            
            
            % Selection de l'ordre du system
            %         d = dir
            %         String_list = {d.name} % Pour utilisateur avancer du logiciel
            String_list = {'Ordre1','Ordre2','Ordre3','Ordre4','Ordre4 - State Space'};
            [Selection,ok] = listdlg('PromptString','Quelle est l''ordre du système auquel le signal appartient',...
                'SelectionMode','single',...
                'ListString',String_list);
            if (~ok)
                
                disp('Opération annulée')
                close(figure_handle)
                
            else
                choix_ordre = String_list{Selection};
                disp(['Sélection de l''utilisateur : ', choix_ordre])
                
                switch choix_ordre
                    case String_list{1}
                        fHandle = @SysOrdre1;
                        X0 = [8,0.2];
                        Param = {'Km', 'Tau'};
                        Nature_signal= 'Vitesse';
                    case String_list{2}
                        fHandle = @SysOrdre2;
                        X0 = [8,0.2];
                        Param = {'Km', 'Tau'};
                        Nature_signal= 'Position';
                    case String_list{3}
                        fHandle = @SysOrdre3;
                        X0 = [6807,4.50, 177,4.23];
                        Param = {'G', 'a', 'b', 'c'};
                        Nature_signal= 'Vitesse';
                        
                    case String_list{4}
                        fHandle = @SysOrdre4;
                        X0 = [6807,4.50, 177,4.23];
                        Param = {'G', 'a', 'b', 'c'};
                        Nature_signal= 'Position';
                        
                    case String_list{5}
                        fHandle = @SysOrdre4ss;
                        X0 = [ 11.77,0.21,0.17, 0.31, 0.0020,0.0014];
                        Param = {'Km','Kr', 'Tau1', 'Tau2', 'I1', 'I2'};
                        Nature_signal= 'Position';
                        
                end
                
                close(figure_handle)
                
                
                % Minisation de l'erreur carree
                [X,fval] = fminsearch(@NormMeanSquareError,X0);
                
                disp(['Normalized Mean Square Error : ', num2str(fval)])
                
                [y, t] = fHandle(X);
                
                % Affichage comparatif
                Figure_Results = figure;
                plot(Temps, signal_exp, t, y)
                title('Comparatif : réel vs approximation')
                ylabel(Nature_signal)
                xlabel('Temps')
                legend('Expérimental', 'Simulé')
                
                Param(2,:) =  num2cell(X);
                Param(:,end+1) = {'NMSE'; fval};
                
                msgbox(sprintf('%s : %.3f \n', Param{:}),'Résultats Identification')
                
            end
        end
        
    end
    
end


