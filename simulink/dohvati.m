function data= dohvati()
%slijedeće naredbe prebacuju matrice podataka s Taraget računala na
%nadzorno pomoću naredbe SCP te ih spremaju u
%working_directory/simulink/Data

global working_directory
%path=[working_directory,'/simulink/Data'];
path = 'Data'
unix(sprintf('scp root@161.53.68.185:q.mat %s/q.mat',path))
unix(sprintf('scp root@161.53.68.185:dq.mat %s/dq.mat',path));
unix(sprintf('scp root@161.53.68.185:u.mat %s/u.mat',path))
unix(sprintf('scp root@161.53.68.185:q_ref.mat %s/q_ref.mat',path))
unix(sprintf('scp root@161.53.68.185:dq_ref.mat %s/dq_ref.mat',path));

% load (sprintf('%s/q.mat',path));
% load (sprintf('%s/dq.mat',path));
% load (sprintf('%s/u.mat',path));
% load (sprintf('%s/q_ref.mat',path));
% load (sprintf('%s/dq_ref.mat',path));

load ('Data/q.mat');
load ('Data/dq.mat');
load ('Data/u.mat');
load ('Data/q_ref.mat');
load ('Data/dq_ref.mat');

data.t = q(1,:)';
data.q1 = q(2,:)';
data.q1_ref = q_ref(2,:)';
data.dq1 = dq(2,:)';
data.dq1_ref = dq_ref(2,:)';
data.u1 = u(2,:)';

end