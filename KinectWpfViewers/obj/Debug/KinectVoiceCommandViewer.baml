   M S B A M L   `   `   `  ����^  ZMicrosoft.Samples.Kinect.WpfViewers, Version=0.0.0.0, Culture=neutral, PublicKeyToken=nullY1clr-namespace:Microsoft.Samples.Kinect.WpfViewers#Microsoft.Samples.Kinect.WpfViewers  5      B    <Microsoft.Samples.Kinect.WpfViewers.KinectVoiceCommandViewer   5      R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5      R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5      Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5      @d2http://schemas.microsoft.com/expression/blend/2008     5      @kv1clr-namespace:Microsoft.Samples.Kinect.WpfViewers     5      .��5	      � $��30��6	   $	��300��6   
���   �  � ColumnDefinitions  5
   
   �� 5      $
��140* �6   �� 5      $
��500* �6   �� 5      $
��160* �6   .+�5   
   �� -   5        $��textBlockModeFeedback��5   `   $��0��6   $
��Left=�6$   $��14(�6?   
�������$��Say Kinect to activate...��6}   $	��6,4q�6�   $	��Top=�6�   �� 5   
   -   5        $��textBlockCommandFeedback��5   `   $��1��6   $
��Left=�6$   $��14(�6?   
�������$�� ��6�   $	��6,4q�6�   $	��Top=�6�   �� 5   
   -   5        $��textBlockActionFeedback��5   `   $��2��6   $
��Left=�6$   $��14(�6?   
�������$�� ��6   $	��6,4q�6�   $	��Top=�6�   ?   9Microsoft.Samples.Kinect.WpfViewers.KinectSpeechCommander  5   
   -   5          Name$ kinectSpeechCommander��5   #      ActiveListeningTimeout$ 5��5   #   !   DefaultConfidenceThreshold$
 0.50��5   #   !   DefaultListeningGrammarUri$k epack://application:,,,/Microsoft.Samples.Kinect.WpfViewers;component/SpeechGrammars/SpeechGrammar.xml?�5   #   "   DefaultListeningGrammarRule$ defaultListen��6�       ActiveListeningGrammarUri$k epack://application:,,,/Microsoft.Samples.Kinect.WpfViewers;component/SpeechGrammars/SpeechGrammar.xml?�5   #   !   ActiveListeningGrammarRule$ activeListen��6�   5      5      